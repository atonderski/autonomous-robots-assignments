FROM ubuntu:16.04 as builder
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir /tmp/build && \
    cd /tmp/build && \
    cmake -D CMAKE_BUILD_TYPE=Release /opt/sources && \
    make && make test && cp helloworld /tmp

# Deploy.
FROM ubuntu:16.04
WORKDIR /opt
COPY --from=builder /tmp/helloworld . 
ENTRYPOINT ["/opt/helloworld"]
