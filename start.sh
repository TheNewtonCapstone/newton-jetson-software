# !/bin/bash
work_dir=$(pwd)
# exec docker compose up -d
function run_container(){
    docker run -it --net=host \
        -e DISPLAY=$DISPLAY \
        -v /dev:/dev \
        --device-cgroup-rule='c *:* rmw' \
        --device=/dev \
        -v /${work_dir}:/${work_dir}\
        thenewtoncapstone/newton:l4t-r36.4.0 \
    bin/bash
}