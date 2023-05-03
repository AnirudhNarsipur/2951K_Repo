# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
# - This is the script from the forum online

XAUTH=$XAUTHORITY

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."
docker stop baxter || true 
docker rm baxter || true 
docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged
    --runtime=nvidia \ 
    baxter \     
    bash

echo "Done."
#docker create -i -t --name baxter --network=host --privileged baxter --env='DISPLAY' --volume=/home/julia/.Xauthority:/root/.Xauthority:rw --net=host
