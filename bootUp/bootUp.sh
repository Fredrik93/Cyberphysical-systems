#make file executable : $ chmod +x bootUp.sh
#if you would want to run the commands without the bootup file cd into same folder and then enter them without x-terminal-emulator -e as pre fix and & in the end one by one, don't forget xhost +

xhost + &
x-terminal-emulator -e docker run --rm --init --net=host --name=opendlv-vehicle-view -v $PWD:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chalmersrevere/opendlv-vehicle-view-multi:v0.0.60 &
x-terminal-emulator -e  docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.3 --cid=253 --name=img

#Commands to run microservice 

#docker build -f Dockerfile -t group07:latest .
#docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp group07:latest --cid=253 --name=img --width=640 --height=480 --verbose 




#link : https://askubuntu.com/questions/972343/how-do-i-create-a-batch-file-and-run-it
#x-terminal-emulator -e /home/user/hacky your-script optional arguments here