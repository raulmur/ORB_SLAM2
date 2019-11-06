dir="`pwd`"

echo '['

pushd /usr/include/eigen3/Eigen &> /dev/null
for i in *; do echo "  { include: ['@<Eigen/src/$i/.*>', 'private', '<Eigen/$i>', 'public'] },"; done | sed s/\'/\"/g
popd &>/dev/null

echo

pushd /usr/include/opencv4/opencv2 &>/dev/null
for i in *.hpp; do echo "  { include: ['@<opencv2/${i%.hpp}/.*.hpp>', 'private', '<opencv2/$i>', 'public'] },"; done | sed s/\'/\"/g
popd &> /dev/null

echo '  { include: ["<this is a dummy that does not exist>", "private", "<blah>", "public"] }'
echo ']'
