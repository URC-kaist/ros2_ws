cd ~/mr2-stack
set -a; source .env; set +a
sudo systemctl start chrony.service nginx.service
chronyc -n tracking
