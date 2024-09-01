FROM jeguzzi/navground:ubuntu24.04

RUN source env/bin/activate \
	&& pip install websockets cairosvg moviepy matplotlib pandas\
    && pip install --no-cache --upgrade pip \
    && pip install --no-cache notebook jupyterlab

RUN apt-get update && apt-get install -y \
    libcairo2 \
   && rm -rf /var/lib/apt/lists/*

ARG NB_USER=ubuntu
ARG NB_UID=1000
ENV USER=${NB_USER}
ENV NB_UID=${NB_UID}
ENV HOME=/home/${NB_USER}

COPY docs/tutorials ${HOME}/tutorials
USER root
RUN chown -R ${NB_UID} ${HOME}
USER ${NB_USER}

WORKDIR /home/${NB_USER}