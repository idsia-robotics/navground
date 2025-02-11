FROM ubuntu:latest

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y curl wget \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp && cd /tmp \
    # && if [[ `uname -m` -eq "aarch64" ]]; then ARCH=arm64; else ARCH=amd64; fi \
    && ARCH=`dpkg --print-architecture` \
    && curl -s https://api.github.com/repos/idsia-robotics/navground/releases/latest \
       | grep "browser_download_url.*$ARCH.deb" \
       | cut -d : -f 2,3 \
       | tr -d \" \
       | wget -qi -

RUN apt-get update && apt-get install -y /tmp/*.deb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv navground_venv

RUN source navground_venv/bin/activate \
    && cd /tmp \
    && ARCH=`uname -m` \
    && PY=`python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"` \
    && curl -s https://api.github.com/repos/idsia-robotics/navground/releases/latest \
       | grep "browser_download_url.*cp$PY-linux_$ARCH.whl" \
       | cut -d : -f 2,3 \
       | tr -d \" \
       | wget -qi -

RUN source navground_venv/bin/activate \
    && wheels=( /tmp/*.whl ) \
    && pip install ${wheels[0]}[all]

COPY ./entrypoint_release.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
