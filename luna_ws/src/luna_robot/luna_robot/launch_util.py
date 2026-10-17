"""Shared by the launch files (kept in the package so every launch file can import it)."""

# Starts a C++ node with SIGINT ignored. rclcpp still installs its own handler, and puts SIG_IGN back when it shuts
# down, so a second SIGINT during teardown can't kill the node with -2. launch sends that second SIGINT when stdin
# is not a terminal (docker without -t, scripts, services) after the process group already got one.
IGNORE_LATE_SIGINT = """sh -c 'trap "" INT; exec "$0" "$@"'"""
