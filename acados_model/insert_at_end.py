import os
import shutil

def main():
    current_path = os.getcwd();
    file_list = ['acados_solver_Wheelchair.c', 'acados_sim_solver_Wheelchair.c',
                 'acados_solver_Wheelchair.h', 'acados_sim_solver_Wheelchair.h',
                ]
    os.makedirs('solver',exist_ok=True)
    for item in file_list:
        shutil.copy2(current_path+'/c_generated_code/'+item,current_path+'/solver/'+item)

    sub_dirs_list = next(os.walk(os.getcwd()+'/c_generated_code'))[1];
    for sub_dir in sub_dirs_list:
        os.makedirs('solver/'+sub_dir,exist_ok=True)
        sub_list = next(os.walk(os.getcwd()+'/c_generated_code/'+sub_dir))[2];
        for item in sub_list:
            if item[-2:]!='.o':
                shutil.copy2(current_path+'/c_generated_code/'+sub_dir+'/'+item,current_path+'/solver/'+sub_dir+'/'+item)

if __name__=="__main__":
    main()
