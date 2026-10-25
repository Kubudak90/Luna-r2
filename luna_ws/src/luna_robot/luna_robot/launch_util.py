"""Shared by the launch files (kept in the package so every launch file can import it)."""

# Starts a C++ node with SIGINT ignored. rclcpp still installs its own handler, and puts SIG_IGN back when it shuts
# down, so a second SIGINT during teardown can't kill the node with -2. launch sends that second SIGINT when stdin
# is not a terminal (docker without -t, scripts, services) after the process group already got one.
IGNORE_LATE_SIGINT = """sh -c 'trap "" INT; exec "$0" "$@"'"""

# Starts a non-ROS server (whisper-server, Piper's HTTP server) so that launch's shutdown SIGINT ends it as a clean
# exit: the wrapper passes it on as SIGTERM and exits 0, instead of launch reporting "process has died" (-2) for a
# server without its own SIGINT handling (both real ones exited 0 on one SIGINT on a Mac, 2026-10-25). The server
# runs as a background job, so the terminal's Ctrl-C to the whole group does not reach it a second time. If the
# server stops by itself, its own exit status comes through.
STOP_AS_CLEAN_EXIT = r"""sh -c 'trap "kill -TERM \$child 2>/dev/null; wait \$child; exit 0" INT TERM; "$0" "$@" & child=$!; wait $child'"""
