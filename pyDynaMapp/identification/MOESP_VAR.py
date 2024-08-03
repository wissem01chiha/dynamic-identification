import numpy as np 
import matplotlib.pyplot as plt

class MOESP:
    """ 
    https://people.duke.edu/~hpgavin/SystemID/References/Verhaegen-IJC-1992a.pdf
    http://www.diag.uniroma1.it/~batti/papers_ifsd/8.pdf
    https://people.duke.edu/~hpgavin/SystemID/References/DeCock-SubspaceID-EOLLS-2003.pdf
    https://people.duke.edu/~hpgavin/SystemID/References/DeCock-SubspaceID-EOLLS-2003.pdf
    """
    def __init__(self) -> None:
        pass




""" 
% MOESP_VAR FUNCTION

function [A,B,C,D,n]=moespvaresv(u,y,R)

%a=input('Insira o valor de a: '); % window data size as an input


  A=[];
  B=[];
  C=[];
  D=[];

for i=a:a:1000   


    [A(:,(2*i)/a-1:(2*i)/a),B(:,(2*i)/a-1:(2*i)/a),C(:,(2*i)/a-1:(2*i)/a),D(:,(2*i)/a-1:(2*i)/a)]=moespar(u(:,i-a+1:i),y(:,i-a+1:i),R);

    % A(:,(2*i)/a-1:(2*i)/a) i=100 >>> A(:,1:2) == A(:,impar:par)


end

n=size(A,1);




    """