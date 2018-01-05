argument1="Maqui"
argument2="Vocabulary/ORBvoc.txt"
argument3="Examples/ROS/ORB_SLAM2/maqui_front.yaml"


if [ "$1" == gdb ]; then
   gdb --args Examples/ROS/ORB_SLAM2/Maqui ${argument2} ${argument3}
fi

rosrun "ORB_SLAM2" ${argument1} ${argument2} ${argument3}

# using gdb

# enter "run" to run the executable main executed with "gdb main"
# enter backtrace to see a backtrace of where the gdb was with executing until error
# enter bt n for a backtrace with n innermost frames
# enter breakpoint <command> to set a breakpoint at a specific command where the execution failed
# enter condition #breakpointnumber to set a condition when to break at this breakpoint. 

# optional
# enter "set print pretty on"
# control + L to clear screen output

# more info see https://www.cs.cmu.edu/~gilpin/tutorial/
