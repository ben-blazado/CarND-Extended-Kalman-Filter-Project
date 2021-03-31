rsync -a /mnt/c/users/owner/documents/git/carnd-extended-kalman-filter-project/src/. ~/carnd-extended-kalman-filter-project/src
cmake ..
make |& grep "error"
cp /mnt/c/users/owner/documents/git/carnd-extended-kalman-filter-project/readme.md ../README.md

