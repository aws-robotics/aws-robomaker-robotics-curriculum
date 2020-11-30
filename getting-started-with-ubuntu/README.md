# Getting Started with Ubuntu

This is the content for the AWS RoboMaker Course 1a - Introduction to Linux and ROS.

## Setting up the environment
The content is generated using Jekyll. Take a look at the instructions for setting up the system to generate the site [here](https://jekyllrb.com/docs/).

## Test it locally

### With Jekyll installed locally

Currently, there are two styles:
1. With some style, meant as a standalone website:

        bundle exec jekyll serve --trace --config _config.yml,_config-standalone.yml

2. Barebone, meant to be as a way to populate the Canvas course.

        bundle exec jekyll serve --trace --config _config.yml,_config-canvas.yml

### Using a Docker container

Start a Jekyll container:

```
docker pull jekyll/jekyll
docker run --rm --volume="$PWD:/srv/jekyll" -p 4000:4000 -it jekyll/jekyll /bin/bash
```

In the running container, start the Jekyll server, then access via http://localhost:4000:

1. With some style, meant as a standalone website:

        bash-5.0# bundle install
        bash-5.0# bundle exec jekyll serve --host 0.0.0.0 --trace --config _config.yml,_config-standalone.yml

2. Barebone, meant to be as a way to populate the Canvas course.

        bash-5.0# bundle install
        bash-5.0# bundle exec jekyll serve --host 0.0.0.0 --trace --config _config.yml,_config-canvas.yml

## Attribution & Licensing

Materials and text substantially authored by Alberto Quattrini Li. Copyright 2020 by Amazon.com, Inc. or its affiliates. Licensed CC-BY-4.0-I
