FROM ubuntu:17.10
COPY scripts/linux /tmp
RUN /tmp/bootstrap.sh \
    && rm -rf /root/packages
VOLUME /root/orb-slam2
