
xhost +local:docker

docker run -it \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --net=host \
    --privileged \
    --name rva_container \
    --mount type=bind,source=/home/eps/Documentos/robotica_y_cv/rva/rva_exchange,target=/home/rva_container/rva_exchange \
    rva_container \
    bash
    
docker rm rva_container
