#!/bin/sh
# REQUIRES NETWORK CONNECTIVITY
mapviz_setup(){
	mkdir -p ~/mapproxy

	if [ ! -f ~/mapproxy/mapproxy.yaml ]; then
		pushd ~/mapproxy
		echo "downloading mapproxy.yaml"
		wget https://raw.githubusercontent.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite/master/mapproxy.yaml
		popd
	fi

	sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
}

mapviz_setup
