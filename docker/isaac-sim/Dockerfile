FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Xvfb provides a virtual X11 display so that carb plugin startup ordering
# works correctly in headless mode. Without it, the RTX radar model plugins
# (wpm_dmatapprox, dmatapprox) crash in carbOnPluginPreStartup because
# IMaterialReaderFactory / IProfileReaderFactory are not yet registered.
# NVIDIA's own documentation recommends xvfb-run for headless RTX sensor use.
USER root
RUN apt-get update && apt-get install -y --no-install-recommends xvfb x11-utils \
    && rm -rf /var/lib/apt/lists/*
USER isaac-sim

# Install Python dependencies
RUN /isaac-sim/python.sh -m pip install pyyaml

# Copy application files
COPY isaac_sim_scripts/ /app/isaac_sim_scripts/
COPY config/ /app/config/

COPY --chmod=755 docker/isaac-sim/entrypoint.sh /entrypoint.sh

WORKDIR /isaac-sim

ENTRYPOINT ["/entrypoint.sh"]
