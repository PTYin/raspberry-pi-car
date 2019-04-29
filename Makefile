install:
	@sudo apt-get install fswebcam
	@npm install .
	gcc car.c -o car -lwiringPi -lm

specs := $(shell find ./tests -name '*.test.js' ! -path "*node_modules/*")
reporter = spec
opts =
test:
	@rm -fr tests/_site
	@node_modules/.bin/mocha --reporter ${reporter} ${opts} ${specs}

server:
	@node app.js
