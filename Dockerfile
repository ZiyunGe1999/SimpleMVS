FROM python:3.8

RUN apt update
RUN apt install -y libgoogle-glog-dev libeigen3-dev
