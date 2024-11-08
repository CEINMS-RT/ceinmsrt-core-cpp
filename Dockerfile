#
# This Dockerfile can be used to buid, run and debug the Ceinms project.
# While using Docker you do you not need to prepare your own system with e.g.
# OpenSim or other dependencies.
#

FROM be1et/ceinms-platform:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install tools needed for remote buid/run/debug
RUN apt-get update -q && \
    apt-get install -y ssh rsync gdb gdbserver --no-install-recommends


# Prepare a trival root login
RUN mkdir /var/run/sshd && \
    echo 'root:Docker' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Add alternative debugger login
RUN useradd -ms /bin/bash debugger && \
    echo 'debugger:pwd' | chpasswd

# 22 for ssh server. 7777 for gdb server.
EXPOSE 22 7777

# Run SSH server
CMD ["/usr/sbin/sshd", "-D"]
