pip uninstall informer -y
mkdir _tmp
cd _tmp
git clone https://github.com/IamWangYunKai/informer.git
cd informer
python setup.py install
cd ../..
rm -rf _tmp