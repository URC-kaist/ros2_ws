cd ~/mr2-stack
set -a; source .env; set +a
sudo systemctl start chrony.service
chronyc -n sources -v
