# FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu20.04 as amd64_base

# FROM nvcr.io/nvidia/l4t-base:r32.5.0 as arm64_base

# FROM ${TARGETARCH}_base

ARG BASE_IMAGE

FROM ${BASE_IMAGE}

RUN ls /usr/local && echo `nproc`

RUN apt-get update \
    && apt-get install -y --no-install-recommends make g++ lsb-release

RUN lsb_release -a && nproc

COPY ./cuda-samples /root/samples

ENV CUDA_PATH "/usr/local/cuda"

WORKDIR /root/samples
RUN sed 's/SMS ?= 50 52 60 61 70 75 80 86 89 90/SMS ?= 50 52 60 61 70 75 80 86 /g' -i /root/samples/Samples/1_Utilities/deviceQuery/Makefile 
RUN sed 's/SMS ?= 53 61 70 72 75 80 86 87 90/SMS ?= 53 61 70 72 75 /g' -i /root/samples/Samples/1_Utilities/deviceQuery/Makefile
RUN sed 's/ALL_CCFLAGS += --threads 0 --std=c++11/ALL_CCFLAGS += --std=c++11 /g' -i /root/samples/Samples/1_Utilities/deviceQuery/Makefile
RUN cd /root/samples/Samples/1_Utilities/deviceQuery \
    && make -j`nproc`

CMD ["/root/samples/Samples/1_Utilities/deviceQuery/deviceQuery"]
