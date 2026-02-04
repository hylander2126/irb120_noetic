import os

# Set the path to your folder here
folder_path = '../experiments/20260131_exp/'

for filename in os.listdir(folder_path):
    if filename.endswith("_fit_summary.txt"):
        parts = filename.split('_')
        
        # Check if the filename matches your expected pattern
        # parts[0] is Date, parts[1] is Time
        if len(parts) >= 2:
            # Remove the second element (the time)
            new_parts = [parts[0]] + parts[2:]
            new_name = "_".join(new_parts)
            
            old_file = os.path.join(folder_path, filename)
            new_file = os.path.join(folder_path, new_name)
            
            os.rename(old_file, new_file)
            print(f"Renamed: {filename} -> {new_name}")