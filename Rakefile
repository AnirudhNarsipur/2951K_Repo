task :build do
  sh "docker build . --build-arg hostuid=$(id -u) --build-arg hostgid=$(id -g) --build-arg hostuser=$(whoami) --build-arg hostgroup=$(whoami) --tag baxter --build-arg hostname=$(hostname)"
end

task :create do
  sh "docker stop baxter || true"
  sh "docker rm baxter || true"
  sh "docker create -i -t --name baxter --network=host --privileged baxter"
  #-env='DISPLAY' -volume=/home/julia/.Xauthority:/root/.Xauthority:rw"
end

task :start do
  sh "docker start --attach -i baxter"
end

task :run do
  sh "../nvidia-docker/nvidia-docker run --rm -ti --net=host --privileged --volume='/home/julia/.Xauthority:/root/.Xauthority:rw' --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --env='QT_X11_NO_MITSHM=1' --env  LIBGL_ALWAYS_INDIRECT=1  baxter"
  
end
