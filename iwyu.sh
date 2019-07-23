mkdir -p build
cd build

# create compile_commands.json
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "Running include-what-you-use. This can take a very long time (5-15 minutes)."
iwyu-tool -p . -- -Xiwyu --mapping_file=../iwyu.imp | tee iwyu.out

echo "Now fixing the includes"
iwyu-fix-includes -m < iwyu.out
