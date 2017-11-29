#/bin/bash

current_dir=$(dirname $0)
project_dir=${current_dir}/../..
cd ${project_dir}

$(git whatchanged HEAD^! | grep docker/Dockerfile > /dev/null)
dockerfile_status=$?

$(git whatchanged HEAD^! | grep scripts/linux/bootstrap.sh > /dev/null)
bootstrap_status=$?

new_image_status=1
t1=${DOCKER_USER}/orb-slam2:${TRAVIS_BUILD_NUMBER}
t2=${DOCKER_USER}/orb-slam2:latest

echo bootstrap_status=${bootstrap_status}
echo dockerfile_status=${dockerfile_status}

if [[ ${dockerfile_status} == "0" || ${bootstrap_status} == "0" ]]
then
    echo "New docker image is required, building new image"
    docker build --compress --tag=${t1} --tag=${t2} --file=docker/Dockerfile .
    new_image_status=$?
else
    echo "New docker image is not required"
fi

./docker/build-from-docker.sh
build_status=$?

if [[ ${new_image_status} == "0" &&  ${build_status} == "0" ]]
then
    if [[ ${TRAVIS_BRANCH} == "master" && ${TRAVIS_PULL_REQUEST} == "false" ]]
    then
        echo "Uploading docker image to Docker hub"
        docker login --username=$DOCKER_USER --password=$DOCKER_PASS
        docker push ${t1}
        docker push ${t2}
      else
        echo "Not a commit on 'master', Docker image is not pushed to Docker hub"
    fi
fi
