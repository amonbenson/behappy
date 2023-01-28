FROM gitpod/workspace-base

USER gitpod

RUN wget -O miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-py310_22.11.1-1-Linux-x86_64.sh \
    && chmod +x miniconda.sh \
    && ./miniconda.sh -u -b \
    rm miniconda.sh
