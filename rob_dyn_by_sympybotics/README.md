# An example for robot dynamics 
Tow-link robot example for Modeling and Identification of Robot Dynamics based on [SymPyBotics](https://github.com/cdsousa/SymPyBotics)  


## Environment  
python>=3.6  

### Packages  
pip install sympy==0.7.5  

## Usage  
### example
```
python Two_link_dyn_iden.py
```  
Compare the "output.csv" with "test_tor_q_dq_ddq.csv".
### data generation
To get "test_tor_q_dq_ddq.csv" and "iden_tor_q_dq_ddq.csv" in matlab:  
./gen_test_data/gen_twolink_dyn_data.m

## TODO
Fix [AttributeError: module 'sympy' has no attribute 'iterables'](https://github.com/cdsousa/SymPyBotics/issues/32) with sympy>=1.7.1
