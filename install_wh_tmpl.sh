WH_TMPL_PATH=`pwd`
LINK_PATH=~/.bin_wh_tmpl

mkdir -p $LINK_PATH

pushd $LINK_PATH
ln -s $WH_TMPL_PATH/cli_menu.py
ln -s $WH_TMPL_PATH/wh_tmpl.py
ln -s $WH_TMPL_PATH/.tmpl
popd
echo PATH=\$PATH:$LINK_PATH >> ~/.bashrc
