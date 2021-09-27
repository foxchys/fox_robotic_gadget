import sympy
# import math
import re
import sympybotics
import numpy as np


class DynIdentify(object):
    """Class for Robotic Dynamic Identification based on sympybotics"""
    """by foxchys Email:chy_s@outlook.com"""

    # input:
    # robot_name:'my_robot',
    # dh_param:[(alpha, a, d, thetax),(),()...],
    # dh_convention:'standard' or 'modified'
    # gravityacc: [0.0, 0.0, -9.81]
    # frictionmodel = {'Coulomb', 'viscous'}
    def __init__(self, robot_name, dh_param, dh_convention, gravityacc, friction_model=None, ):
        self.rbtdef = sympybotics.RobotDef(robot_name,  # robot name
                                           dh_param,  # (alpha, a, d, theta)
                                           dh_convention  # either 'standard' or 'modified'
                                           )
        if friction_model is not None:
            self.rbtdef.frictionmodel = friction_model
        self.rbt = sympybotics.RobotDynCode(self.rbtdef, verbose=True)
        self.rbt.dyn.gen_all()
        self.rbt.gravityacc = sympy.Matrix(gravityacc)
        self.rbt.calc_base_parms()
        self.baseparams_value = np.zeros(len(self.rbt.dyn.baseparms), np.float64)
        self.rob_dof = self.rbtdef.dof
        return

    def getsym_dynparams(self):
        return self.rbtdef.dynparms()

    def getsym_hmatrix(self):
        return self.rbt.dyn.H

    def getsym_baseparams(self):
        return self.rbt.dyn.baseparms

    def getsym_h_basematrix(self):
        # print("self.rbt.dyn.Pb:")
        # print(self.rbt.dyn.Pb)
        # print("self.rbt.dyn.Pb:")
        return self.rbt.dyn.H * self.rbt.dyn.Pb

    def get_code_h_func(self, code_type='python'):
        if code_type == 'matlab':
            return self._convert_pyhfunc2matlabfunc(sympybotics.robotcodegen.robot_code_to_func(
                'python', self.rbt.H_code, 'H', 'H_rbt', self.rbtdef), 'H')
        else:
            return sympybotics.robotcodegen.robot_code_to_func(
                code_type, self.rbt.H_code, 'H', 'H_rbt', self.rbtdef)

    def _get_hfunc(self):
        # h_func_def = sympybotics.robotcodegen.robot_code_to_func(
        #     'python', self.rbt.H_code, 'H', 'H_rbt', self.rbtdef)
        h_func_def = self.get_code_h_func('python')
        h_func_def = h_func_def.replace('sign', 'np.sign')
        h_func_def = h_func_def.replace('sin', 'np.sin')
        h_func_def = h_func_def.replace('cos', 'np.cos')
        # global sin, cos, sign
        # sin = np.sin
        # cos = np.cos
        # sign = np.sign
        exec(h_func_def, globals(), locals())
        return locals()['H_rbt']

    def get_h_matrix(self, q, dq, ddq):
        q = q.flatten()
        dq = dq.flatten()
        ddq = ddq.flatten()
        h_func = self._get_hfunc()
        hb = h_func(q, dq, ddq)
        return np.matrix(hb).reshape(self.rbt.dof, len(self.rbt.dyn.dynparms)).astype(np.float64)

    def _convert_pyhfunc2matlabfunc(self, in_pyfunc, func_ouput_name='Hb'):
        matlab_func = in_pyfunc
        # re.findall('\[\d*\.?\d*\]',in_pyfunc)

        matlab_func = re.sub(r'def', 'function [%s] =' % func_ouput_name, matlab_func, 1)
        matlab_func = re.sub(func_ouput_name+r'\s=\s\[0]\*\d*',
                             lambda x: r'%s =zeros(%d,%d)' %
                                       (func_ouput_name,
                                        self.rob_dof,
                                        int(x.group()[(x.group().find('*')+1):len(x.group())])/self.rob_dof),
                             matlab_func, 1)
        # replace '[n]' to '(n+1)'
        matlab_func = re.sub(r'\[\d*]',
                             lambda x: r'(%d)' % (int(x.group()[1:(len(x.group()) - 1)]) + 1),
                             matlab_func)
        # replace '**n' to '^n'
        matlab_func = re.sub(r'\*\*\d*',
                             lambda x: r'^%d' % (int(x.group()[2:(len(x.group()))])),
                             matlab_func)
        matlab_func = re.sub(r'#', '%', matlab_func)
        matlab_func = re.sub(r'\n', ';\n', matlab_func)
        matlab_func = re.sub(r';\n;', ';\n', matlab_func, 1)
        matlab_func = re.sub(r'\):;', r')', matlab_func, 1)
        matlab_func = re.sub(r'\s*return.*', r'\nend', matlab_func, 1)
        return matlab_func

    def get_code_hbase_func(self, code_type='python'):
        if code_type == 'matlab':
            return self._convert_pyhfunc2matlabfunc(sympybotics.robotcodegen.robot_code_to_func(
                'python', self.rbt.Hb_code, 'Hb', 'Hb_rbt', self.rbtdef), 'Hb')
        else:
            return sympybotics.robotcodegen.robot_code_to_func(
                code_type, self.rbt.Hb_code, 'Hb', 'Hb_rbt', self.rbtdef)

    def _get_hbasefunc(self):
        # h_func_def = sympybotics.robotcodegen.robot_code_to_func(
        #     'python', self.rbt.Hb_code, 'Hb', 'Hb_rbt', self.rbtdef)
        h_func_def = self.get_code_hbase_func('python')
        h_func_def = h_func_def.replace('sign', 'np.sign')
        h_func_def = h_func_def.replace('sin', 'np.sin')
        h_func_def = h_func_def.replace('cos', 'np.cos')
        # global sin, cos, sign
        # sin = np.sin
        # cos = np.cos
        # sign = np.sign
        exec(h_func_def, globals(), locals())
        return locals()['Hb_rbt']

    def get_hbase_matrix(self, q, dq, ddq):
        q = q.flatten()
        dq = dq.flatten()
        ddq = ddq.flatten()
        hb_func = self._get_hbasefunc()
        hb = hb_func(q, dq, ddq)
        return np.matrix(hb).reshape(self.rbt.dof, len(self.rbt.dyn.baseparms)).astype(np.float64)

    # get baseparams based on : tor = Hb*baseparams
    def iden_baseparams(self, tor, q, dq, ddq):
        tor = np.mat(tor.copy())
        q = np.mat(q.copy())
        dq = np.mat(dq.copy())
        ddq = np.mat(ddq.copy())
        if tor.shape[0] == 1:
            tor = tor.T
            q = q.T
            dq = dq.T
            ddq = ddq.T
        hb_whole_matrix = np.zeros((len(tor) * self.rbt.dof, len(self.rbt.dyn.baseparms)), np.float64)
        tor_whol_vec = np.zeros((len(tor) * self.rbt.dof, 1), np.float64)
        for ii in range(0, len(tor), 1):
            i = ii * self.rbt.dof
            hb_whole_matrix[i:i + self.rbt.dof, :] = self.get_hbase_matrix(np.array(q[ii, :]),
                                                                           np.array(dq[ii, :]),
                                                                           np.array(ddq[ii, :])).copy()
            tor_whol_vec[i:i + self.rbt.dof, :] = np.mat(tor[ii, :]).T.copy()
        dyn_param = np.dot(np.dot(np.linalg.inv(np.dot(hb_whole_matrix.T, hb_whole_matrix)), hb_whole_matrix.T),
                           tor_whol_vec)
        self.baseparams_value = dyn_param
        return dyn_param

    # get tor based on : tor = Hb*baseparams
    def get_tor_hbxb(self, q, dq, ddq):
        q = np.mat(q.copy())
        dq = np.mat(dq.copy())
        ddq = np.mat(ddq.copy())
        if q.shape[0] == 1:
            q = q.T
            dq = dq.T
            ddq = ddq.T
        tor = np.zeros((len(q), self.rbt.dof), np.float64)
        # call iden_baseparams() before
        assert np.linalg.norm((self.baseparams_value - np.zeros(len(self.rbt.dyn.baseparms), np.float64))) > 0.000001
        for i in range(0, len(tor), 1):
            tor[i, :] = np.dot(self.get_hbase_matrix(np.array(q[i, :]),
                                                     np.array(dq[i, :]),
                                                     np.array(ddq[i, :])), self.baseparams_value).T
        return tor


def main():
    # # (alpha, a, d, theta)
    # # pi = sympy.pi
    q = sympybotics.robotdef.q
    # # (alpha, a, d, theta)
    dh_param = [(0, 0, 0, q), (0, 4, 0, q)]
    friction_model = {'Coulomb', 'viscous'}
    gravityacc = [0.0, 0.0, -9.81]
    test_dyn_identify = DynIdentify('test_twolink', dh_param, 'modified', gravityacc, friction_model)
    print('sym_dynparams:')
    print(test_dyn_identify.getsym_dynparams())
    print('getsym_hmatrix')
    print(test_dyn_identify.getsym_hmatrix())
    print('getsym_baseparams:')
    print(test_dyn_identify.getsym_baseparams())
    print('getsym_h_basematrix:')
    print(test_dyn_identify.getsym_h_basematrix())
    print('get_python_code_hbase_func:')
    print(test_dyn_identify.get_code_hbase_func())
    # hfunc = test_dyn_identify.get_hfunc()
    # print('hfunc([0, 0], [0, 0], [0, 0])')
    # print(hfunc([0, 0], [0, 0], [0, 0]))
    print('get_hbase_matrix')
    print(test_dyn_identify.get_hbase_matrix(np.array([0.0, 1.0]),
                                             np.array([1.0, -0.0]),
                                             np.array([-0.0, -1.0])))
    f_iden_data = open("iden_tor_q_dq_ddq.csv", "rb")
    iden_tor_p_v_a = np.loadtxt(f_iden_data, delimiter=",", skiprows=0)
    print('iden_baseparams')
    print(test_dyn_identify.iden_baseparams(iden_tor_p_v_a[:, 0:2], iden_tor_p_v_a[:, 2:4],
                                            iden_tor_p_v_a[:, 4:6], iden_tor_p_v_a[:, 6:8]))
    # # test the result
    ft_iden_data = open("test_tor_q_dq_ddq.csv", "rb")
    test_tor_p_v_a = np.loadtxt(ft_iden_data, delimiter=",", skiprows=0)
    # # test_tor_p_v_a = iden_tor_p_v_a
    ft_iden_data.close()
    test_tor = test_dyn_identify.get_tor_hbxb(test_tor_p_v_a[:, 2:4],
                                              test_tor_p_v_a[:, 4:6],
                                              test_tor_p_v_a[:, 6:8])
    np.savetxt('output.csv', test_tor, delimiter=',')
    print('test_tor:')
    print(test_tor)
    return


main()

