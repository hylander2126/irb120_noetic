#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
import numpy as np
import threading

# Used to transform NetFT data to the correct frame
# Listens to /netft_data (WrenchStamped) and republishes to /netft_data_transformed
# with the forces transformed by a fixed rotation

class FTTransformer:
    def __init__(self):
        self.input_topic    = rospy.get_param('~input_topic', '/netft_data')
        self.output_topic   = rospy.get_param('~output_topic', '/netft_data_transformed')
        self.apply_bias     = rospy.get_param('~apply_bias', True)
        self.bias_samples   = 150

        # HACK: Use a static rotation for our specific setup
        self.R              = np.array([[ 0,  0,  1],
                                        [ 1,  0,  0],
                                        [ 0,  1,  0]], dtype=float)
        # Bias
        self.lock           = threading.Lock()
        self.have_biased    = False
        self.bias_f         = np.zeros(3)
        self.bias_t         = np.zeros(3)
        # Bias accumulator
        self._bias_acc_f   = np.zeros(3)
        self._bias_acc_t   = np.zeros(3)
        self._bias_count   = 0
        self._biasing_now  = self.apply_bias and (self.bias_samples > 0)
        # Services
        rospy.Service("~bias_now", Trigger, self.srv_bias_now)
        rospy.Service("~clear_bias", Trigger, self.srv_clear_bias)
        # Pub and Sub
        self.pub           = rospy.Publisher(self.output_topic, WrenchStamped, queue_size=100)
        self.sub           = rospy.Subscriber(self.input_topic, WrenchStamped, self.cb, queue_size=200)
        rospy.loginfo("[ft_force_transformer] Listening to %s, publishing to %s", self.input_topic, self.output_topic)


    def srv_clear_bias(self, _req):
        with self.lock:
            self.have_biased = False
            self._biasing_now = False
            self.bias_f[:] = 0
            self.bias_t[:] = 0
        return TriggerResponse(success=True, message="Cleared bias")
    
    def srv_bias_now(self, _req):
        with self.lock:
            # (Re)start accumulation for bias over bias_samples
            self._bias_acc_f[:] = 0
            self._bias_acc_t[:] = 0
            self._bias_count = 0
            self._biasing_now = True
            self.have_biased = False
        return TriggerResponse(success=True, message="Starting bias accumulation")
    
    def cb(self, msg):
        f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float)
        t = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=float)

        with self.lock:
            if self._biasing_now:
                # Accumulate bias
                self._bias_acc_f += f
                self._bias_acc_t += t
                self._bias_count += 1
                if self._bias_count >= self.bias_samples:
                    # Finalize bias
                    self.bias_f = self._bias_acc_f / float(self._bias_count)
                    self.bias_t = self._bias_acc_t / float(self._bias_count)
                    self.have_biased = True
                    self._biasing_now = False
                    rospy.loginfo("[ft_force_transformer] Biasing complete. Bias force: %s, torque: %s", self.bias_f, self.bias_t)
            # if self.have_biased:
            #     # Subtract bias
            #     transformed_F -= self.bias_f
            #     transformed_T -= self.bias_t

        f_final = f.copy()
        t_final = t.copy()

        # Subtract bias if we have it
        if self.have_biased:
            f_final -= self.bias_f
            t_final -= self.bias_t

        # Apply fixed transform:
        transformed_F = self.R @ f_final
        transformed_T = self.R @ t_final

        # Publish using the same exact message type
        out = WrenchStamped()
        out.header = Header()
        out.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        out.wrench.force.x = transformed_F[0]
        out.wrench.force.y = transformed_F[1]
        out.wrench.force.z = transformed_F[2]
        out.wrench.torque.x = transformed_T[0]
        out.wrench.torque.y = transformed_T[1]
        out.wrench.torque.z = transformed_T[2]

        # Publish the transformed message
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('netft_transformer', anonymous=True)
    node = FTTransformer()
    rospy.on_shutdown(getattr(node, 'shutdown', lambda: None))
    rospy.spin()