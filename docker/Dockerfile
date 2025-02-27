FROM ubuntu:latest

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y wget \
    && wget -qO- https://api.github.com/repos/idsia-robotics/navground/releases/latest \
       | grep "browser_download_url.*`dpkg --print-architecture`.deb" \
       | cut -d : -f 2,3 \
       | tr -d \" \
       | wget -P /tmp  -qi - \
    && apt-get install -y /tmp/*.deb \
    && rm -rf /tmp/*.deb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv navground_venv

RUN source navground_venv/bin/activate \
    && wheels=( /tmp/*.whl ) \
    && pip install --find-links https://api.github.com/repos/idsia-robotics/navground/releases/latest navground[all]

COPY ./entrypoint_release.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
