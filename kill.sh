ps aux|grep "world_node"|sed 's@  *@ @g'|cut -d" " -f2|xargs -i kill -9 {}
ps aux|grep "player_node"|sed 's@  *@ @g'|cut -d" " -f2|xargs -i kill -9 {}
