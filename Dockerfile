FROM ubuntu:17.10
COPY bootstrap_linux.sh /tmp
RUN /tmp/bootstrap_linux.sh \
    && rm -rf /root/packages
VOLUME /root/orb-slam2
