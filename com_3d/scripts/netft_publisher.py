#!/usr/bin/env python3
import os, socket, struct, select, csv, time, math
import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse

"""
Minimal ATI Net F/T RDT UDP reader
- Binds a UDP socket (default port 49152) and parses ATI RDT packets.
- Publishes WrenchStamped on <ns>/wrench (default topic: /wrench).
- Optional CSV logging.
- Optional zeroing service <ns>/zero (averages N samples and subtracts).

RDT packet format (36 bytes, big-endian):
  int32 rdt_seq
  int32 ft_seq
  int32 status
  int32 Fx, Fy, Fz, Tx, Ty, Tz   # counts or engineering units depending on sensor config

If your sensor outputs "counts", provide per-axis scales to convert to N/Nm.
If configured to "engineering units" in the web UI, leave scales at 1.0.
"""

def main():
    # ------------------ params ------------------
    bind_ip     = rospy.get_param("~bind_ip", "0.0.0.0")   # NIC IP that faces the sensor or 0.0.0.0
    port        = int(rospy.get_param("~port", 49152))     # ATI RDT default
    sensor_ip   = rospy.get_param("~sensor_ip", "")        # if set, drop packets not from this IP
    frame_id    = rospy.get_param("~frame_id", "ft_link")
    topic       = rospy.get_param("~topic", "wrench")
    # per-axis scale factors (counts->SI). Default 1.0 (assume engineering units).
    scales      = rospy.get_param("~scales", [1,1,1,1,1,1])  # [Fx,Fy,Fz,Tx,Ty,Tz]
    # optional simple rate cap on publish (Hz). 0/None = publish as fast as we receive.
    pub_rate_hz = float(rospy.get_param("~pub_rate_hz", 0.0))
    # CSV logging
    csv_path    = rospy.get_param("~csv_path", "")         # empty = disabled
    csv_forces_only = bool(rospy.get_param("~csv_forces_only", True))
    csv_flush_every = int(rospy.get_param("~csv_flush_every", 50))
    # zeroing samples
    zero_samples = int(rospy.get_param("~zero_samples", 200))

    # sanity
    if len(scales) != 6:
        rospy.logwarn("~scales must be length 6. Using identity.")
        scales = [1,1,1,1,1,1]
    scales = [float(s) for s in scales]

    # ------------------ ROS setup ------------------
    pub = rospy.Publisher(topic, WrenchStamped, queue_size=200)

    bias = [0.0]*6
    def srv_zero(_req):
        nonlocal bias
        rospy.loginfo("Zeroing F/T with {} samples...".format(zero_samples))
        acc = [0.0]*6
        n = 0
        deadline = time.time() + 5.0  # don’t wait forever if no packets
        while n < zero_samples and not rospy.is_shutdown() and time.time() < deadline:
            r, _, _ = select.select([sock], [], [], 0.5)
            if not r:
                continue
            data, addr = sock.recvfrom(2048)
            if sensor_ip and addr[0] != sensor_ip:
                continue
            parsed = parse_rdt(data)
            if not parsed:
                continue
            ft = [parsed[i+3] for i in range(6)]  # raw
            ft_si = [ft[i]*scales[i] for i in range(6)]
            for i in range(6): acc[i] += ft_si[i]
            n += 1
        if n > 0:
            bias = [acc[i]/n for i in range(6)]
        rospy.loginfo("Zero complete. Bias = " + str(["{:.4f}".format(x) for x in bias]))
        return EmptyResponse()

    rospy.Service("zero", Empty, srv_zero)

    # CSV init if requested
    csv_writer = None
    csv_file = None
    csv_count_since_flush = 0
    if csv_path:
        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        csv_file = open(csv_path, "w", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_forces_only:
            csv_writer.writerow(["sec","nsec","Fx","Fy","Fz"])
        else:
            csv_writer.writerow(["sec","nsec","Fx","Fy","Fz","Tx","Ty","Tz"])
        csv_file.flush()
        rospy.loginfo("CSV logging to {}".format(csv_path))

    # ------------------ Socket ------------------
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # bind to NIC (or 0.0.0.0) and port
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bind_ip, port))
    except Exception as e:
        rospy.logerr("Failed to bind UDP {}:{} : {}".format(bind_ip, port, e))
        return

    rospy.loginfo("Listening RDT on {}:{} (filter sensor_ip='{}')".format(bind_ip, port, sensor_ip or "ANY"))

    # simple publish rate limiting
    next_pub_time = 0.0
    period = 1.0/pub_rate_hz if pub_rate_hz and pub_rate_hz > 0.0 else 0.0

    # ------------------ main loop ------------------
    while not rospy.is_shutdown():
        # wait up to 0.5s for a packet
        r, _, _ = select.select([sock], [], [], 0.5)
        if not r:
            continue
        data, addr = sock.recvfrom(2048)
        if sensor_ip and addr[0] != sensor_ip:
            continue
        parsed = parse_rdt(data)
        if not parsed:
            continue

        # parsed = (rdt_seq, ft_seq, status, Fx, Fy, Fz, Tx, Ty, Tz)
        ft_raw = parsed[3:]
        ft_si = [ft_raw[i]*scales[i] - bias[i] for i in range(6)]

        # rate limit
        now = time.time()
        if period and now < next_pub_time:
            # still log CSV even if we skip publish
            if csv_writer:
                write_csv(csv_writer, csv_file, csv_forces_only, ft_si, csv_count_since_flush)
            continue
        next_pub_time = now + period if period else now

        # publish
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z  = ft_si[0], ft_si[1], ft_si[2]
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = ft_si[3], ft_si[4], ft_si[5]
        pub.publish(msg)

        # CSV
        if csv_writer:
            csv_count_since_flush = write_csv(csv_writer, csv_file, csv_forces_only, ft_si, csv_count_since_flush)

def parse_rdt(data: bytes):
    """Return tuple (rdt_seq, ft_seq, status, Fx, Fy, Fz, Tx, Ty, Tz) or None."""
    # ATI RDT is 9x int32 big-endian = 36 bytes
    if len(data) < 36:
        return None
    try:
        vals = struct.unpack("!9i", data[:36])
        return vals
    except struct.error:
        return None

def write_csv(writer, fobj, forces_only, ft_si, n_since):
    t = rospy.Time.now()
    if forces_only:
        writer.writerow([t.secs, t.nsecs, ft_si[0], ft_si[1], ft_si[2]])
    else:
        writer.writerow([t.secs, t.nsecs] + list(ft_si))
    n_since += 1
    if n_since >= int(rospy.get_param("~csv_flush_every", 50)):
        try:
            fobj.flush()
        except Exception:
            pass
        n_since = 0
    return n_since

if __name__ == "__main__":
    rospy.init_node("netft_rdt_minimal")
    main()
