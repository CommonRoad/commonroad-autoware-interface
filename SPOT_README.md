# Installation of SPOT

1. Clone branch `master` of the SPOT repository `https://gitlab.lrz.de/cps/spot-cpp.git` into `src/universe/autoware.universe/planning/tum_commonroad_planning`.
   This may have already happened, if you have set up the environment from scratch.
2. Navigate to the SPOT repo and modify the setup.py to always build in Debug mode. For example by adding this in lines 69 ff.:

   ```python
   cfg = 'Debug' if self.debug else 'Release'
   cfg = 'Debug'
   build_args = ['--config', cfg]
   ```

3. Install the SPOT requirements with `pip install -r requirements.txt`.
4. Install SPOT with `cd spot` and `pip install .`.
