FROM ubuntu:latest

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y curl \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp && cd /tmp \
    && curl -s https://api.github.com/repos/idsia-robotics/navground/releases/latest \
       | grep "browser_download_url.*deb" \
       | cut -d : -f 2,3 \
       | tr -d \" \
       | wget -qi -

# TODO: download wheel

RUN apt-get update && apt-get install -y /tmp/*.dep \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# TODO: install wheel

# RUN python3 -m venv navground_venv \
#     && source navground_venv/bin/activate \
#     && pip install -U /tmp/*.whl[all]

COPY ./entrypoint_release.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
