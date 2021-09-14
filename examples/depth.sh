#!/bin/bash

exe="./src/depth/depth"
path="../examples"

camera="${path}/intrinsics.js"
params="${path}/params.js"
strat="${path}/strategy.js"
images="${path}/images.js"

${exe} \
	--pcamera "${camera}" \
	--pimages "${images}" \
	--pparams "${params}" \
	--strategy "${strat}" \
	--gui true \
	--verbose true -l 15 \

