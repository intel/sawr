#/bin/sh
echo "Initiating Phase $1 as a background process"
./scripts/init_$1.sh &

