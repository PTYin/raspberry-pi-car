install:
	@sudo apt-get install fswebcam
	@npm install .
	gcc car.c -o car -lwiringPi -lm
	gcc distance.c -o distance -lwiringPi -lm
	gcc carClient.c -o carClient -lwiringPi -lm

specs := $(shell find ./tests -name '*.test.js' ! -path "*node_modules/*")
reporter = spec
opts =
test:
	@rm -fr tests/_site
	@node_modules/.bin/mocha --reporter ${reporter} ${opts} ${specs}

server:
	@node app.js
compile:
	gcc car.c -o car -lwiringPi -lm
	gcc distance.c -o distance -lwiringPi -lm
	gcc carClient.c -o carClient -lwiringPi -lm
