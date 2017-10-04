## open the intel r200 stream with vlc
vlc v4l2:// :v4l2-dev=/dev/video2

vlc v4l2:// :v4l2-dev=/dev/video2 --sout '#transcode{vcodec=mp4v,vb=128}:std{access=mmsh,dst=:8080}'


vlc v4l2:// :v4l2-dev=/dev/video2 :v4l2-width=1920 :v4l2-height=1080 --sout '#transcode{vcodec=mp4v,vb=128}:std{access=mmsh,dst=:8080}'


vlc v4l2:// :v4l2-dev=/dev/video2 :v4l2-width=1920 :v4l2-height=1080 --sout '#transcode{vcodec=mp4v,vb=128,samplerate=44100}:std{access=mmsh,dst=:8080}' -I dummy


## list the modes of the camera
v4l2-ctl --list-formats-ext -d /dev/video2


## ffmpeg commands

#this one has low latency but not perfect:
ffmpeg -y -i /dev/video2 -r 25 -b:v 1000k -c:v libx264 -pass 1 -preset superfast -tune zerolatency     -x264opts crf=20:vbv-maxrate=3000:vbv-bufsize=100:intra-refresh=1:slice-max-size=1500:keyint=30:ref=1  -f mpegts udp://127.0.0.1:8080


#to benchmark the latency

ffmpeg -y -i /dev/video2 -r 30  -c:v libx264  -preset superfast -tune zerolatency     -x264opts crf=20:vbv-maxrate=3000:vbv-bufsize=100:intra-refresh=1:slice-max-size=1500:keyint=30:ref=1  -pix_fmt yuv420p  -f rtp rtp://192.168.178.84:8080


#the ip:port is not the "server:ip" that is listening as one would expect, but the target machine
#where the stream is sent
mplayer -benchmark udp://localhost:8080

#decode with gstreamer
gst-launch-1.0 udpsrc port=8080 !   "application/x-rtp, payload=127" !    rtph264depay !     avdec_h264 !     videoconvert  ! xvimagesink sync=false




