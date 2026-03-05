FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Install Python dependencies
RUN /isaac-sim/python.sh -m pip install pyyaml

# Copy application files
COPY isaac_sim_scripts/ /app/isaac_sim_scripts/
COPY config/ /app/config/

COPY --chmod=755 docker/isaac-sim/entrypoint.sh /entrypoint.sh

WORKDIR /isaac-sim

ENTRYPOINT ["/entrypoint.sh"]
