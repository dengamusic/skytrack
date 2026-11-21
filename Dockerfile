# Builds skytrack, runs the test suite, and ships the tools in a small image.
#
#   docker build -t skytrack .
#   docker run --rm skytrack                                   # simulate, track and score the baseline
#   docker run --rm -v "$PWD/runs:/skytrack/runs" skytrack \
#     python3 tools/two_sites.py                               # any benchmark; results land in ./runs

FROM ubuntu:24.04 AS build
RUN apt-get update \
 && apt-get install -y --no-install-recommends cmake g++ ninja-build libeigen3-dev nlohmann-json3-dev ca-certificates \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY include include
COPY src src
COPY apps apps
COPY tests tests
COPY scenarios scenarios
COPY configs configs
# -Werror here as in CI; the tests must pass for the image to build.
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSKYTRACK_WERROR=ON \
 && cmake --build build \
 && ctest --test-dir build --output-on-failure

FROM ubuntu:24.04
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3 \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /skytrack
COPY --from=build /src/build/skytrack_sim /src/build/skytrack_track /src/build/skytrack_eval \
                  /src/build/skytrack_fuse /src/build/skytrack_cue build/
COPY scenarios scenarios
COPY configs configs
COPY tools tools
ENV PATH="/skytrack/build:${PATH}"
CMD ["sh", "-c", "skytrack_sim scenarios/cuas_baseline.json runs/demo > /dev/null && skytrack_track runs/demo > /dev/null && skytrack_eval runs/demo"]
