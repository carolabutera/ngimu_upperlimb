function[data]= import_data(test)
    if test==1
       data=readmatrix("./validation/test1/IMUvsTIAGO_test1.csv");
    elseif test==2
       data=readmatrix("./validation/test2/IMUvsTIAGO_test2.csv");
    elseif test==3
       data=readmatrix("./validation/test3/IMUvsTIAGO_test3.csv");
    elseif test==4
       data=readmatrix("./validation/test4/IMUvsTIAGO_test4.csv");
    elseif test==5
       data=readmatrix("./validation/test5/IMUvsTIAGO_test5.csv");
    elseif test==6
       data=readmatrix("./validation/test6/IMUvsTIAGO_test6.csv");
    elseif test==7
       data=readmatrix("./validation/test7/IMUvsTIAGO_test7.csv");
    end 
end
