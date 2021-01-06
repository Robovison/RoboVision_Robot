## Installation of Mask RCNN

The creators require Python 3.4, but in our case Python 3.7 was used and it worked fine, but it is not suitable for Python 3.8. Also, the implementation needs other packages that are found inside the repository in the `requirements.txt` file.

1. Clone the repository

   `git clone` [https://github.com/matterport/Mask_RCNN.git](https://github.com/matterport/Mask_RCNN.git)

2. Install dependencies

   `pip install -r requirements.txt`

3. Run setup from the repository root directory

```jsx
pip install setuptools
sudo python setup.py install
```

4. Download pre-trained COCO weights (mask_rcnn_coco.h5) from the [releases page](https://github.com/matterport/Mask_RCNN/releases).

5. (Optional) To train or test on MS COCO install pycocotools from [here](https://github.com/waleedka/coco). There are forks of the original pycocotools with fixes for Python3 and Windows (the official repo doesn't seem to be active anymore).

6. (Optional) Install jupyter notebook to check if the installation was done right. Run the demo.ipynb and check if there are any errors showing.

```jsx
pip install jupyterlab
jupyter notebook samples/demo.ipynb
```



**Errors that might appear**

1. When running `pip3 install -r requirements.txt`.

```bash
pip3 install -r requirements.txt 
Collecting numpy (from -r requirements.txt (line 1))
  Using cached https://files.pythonhosted.org/packages/c5/63/a48648ebc57711348420670bb074998f79828291f68aebfff1642be212ec/numpy-1.19.4.zip
    Complete output from command python setup.py egg_info:
    Traceback (most recent call last):
      File "<string>", line 1, in <module>
      File "/tmp/pip-build-37q07muu/numpy/setup.py", line 68
        f"NumPy {VERSION} may not yet support Python "
                                                     ^
    SyntaxError: invalid syntax
    
    ----------------------------------------
Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-build-37q07muu/numpy/
You are using pip version 8.1.1, however version 20.3.1 is available.
You should consider upgrading via the 'pip install --upgrade pip' command.
```

You might solve this by:

- Download this file: [get_pip.py](get_pip.py) 
- Execute `sudo python3 get_pip.py`
- Install numpy with python3.x, where x < 8 `pip3 install numpy`

2. When running the demo.ipynb (`jupyter notebook samples/demo.ipynb`)

   ```
   ImportError                               Traceback (most recent call last)
   <ipython-input-1-ebe7095df7bb> in <module>
        18 # Import COCO config
        19 sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
   ---> 20 import coco
        21 
        22 get_ipython().run_line_magic('matplotlib', 'inline ')
   
   ~/Mask_RCNN/samples/coco/coco.py in <module>
        40 # If the PR is merged then use the original repo.
        41 # Note: Edit PythonAPI/Makefile and replace "python" with "python3".
   ---> 42 from pycocotools.coco import COCO
        43 from pycocotools.cocoeval import COCOeval
        44 from pycocotools import mask as maskUtils
   
   ImportError: No module named 'pycocotools'
   ```

   You should install pycocotools. `pip3 install pycocotools`

   It works only with specific version so install `pip3 install tensorflow==1.14` and `pip3 install keras==2.3.1` of the specified version , otherwise you will not be able to run it.

 