import subprocess

def close_gazebo_windows():
    # Execute the wmctrl command to list all windows
    proc = subprocess.Popen(['wmctrl', '-l'], stdout=subprocess.PIPE)

    # Read the output of wmctrl command
    output, _ = proc.communicate()

    # Iterate over the lines of output
    for line in output.splitlines():
        # Check if the window title contains 'gzclient'
        if b'gzclient' in line:
            # Extract the window ID from the line
            window_id = line.split()[0]

            # Execute the wmctrl command to close the window
            subprocess.call(['wmctrl', '-ic', window_id])

if __name__ == '__main__':
    close_gazebo_windows()

