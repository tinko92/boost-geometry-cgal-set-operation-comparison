rm -r out/result*.csv

./run.sh difference single > out/results_difference.csv
./run.sh difference multi > out/results_difference_multi.csv
./run.sh union single > out/results_union.csv
./run.sh union multi > out/results_union_multi.csv
./run.sh intersection single > out/results_intersection.csv
./run.sh intersection multi > out/results_intersection_multi.csv
./run.sh sym_difference single > out/results_sym_difference.csv
./run.sh sym_difference multi > out/results_sym_difference_multi.csv

cat header.csv out/results_*.csv > out/results.csv
sed -i 's/\.,/\.0,/g' out/*.svg
sed -i 's/\. /\.0 /g' out/*.svg
rm -f Core_Diagnostics
