FROM aerothon-dependencies:20261014
ENV DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /home/aerothon && chown 1000:1000 /home/aerothon
ENV HOME=/home/aerothon ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 http_proxy= https_proxy= no_proxy=
WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
