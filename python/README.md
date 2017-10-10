Must be installed rbdl-python

     export PYTHONPATH=(RBDL_SO_DIR):$PYTHONPATH

where (RBDL_SO_DIR).

Make a file to set the environment

   # add the following to a file
   RBDL_SO_PATH={..RBDL_SO_DIR..}
   PYTHONPATH=$RBDL_SO_PATH:$PYTHONPATH
   export PYTHONPATH 
