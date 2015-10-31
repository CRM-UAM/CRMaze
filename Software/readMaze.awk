BEGIN{
    j=0;i=0;
}
{
    if(j<27)for(i=1;i<=27;i++)m[j,i-1]=$i;j++;
}
END{
print("{");
for(j=0;j<13;j++){
    printf("{")
    for(i=0;i<13;i++){
        ret = 0;
        nX=i*2+1;
        nY=j*2+1;
        if(m[nY-1,nX]!=1)ret+=1; //no hay pared al norte
        if(m[nY+1,nX]!=1)ret+=2; //no hay pared al sur
        if(m[nY,nX-1]!=1)ret+=8; //no hay pared al oeste
        if(m[nY,nX+1]!=1)ret+=4; //no hay pared al este
        printf("%d",ret);
        if(i<12)printf(",");
    }
    printf("}");
    if(j<12)printf(",");
    printf("\n");
}
print("}\n");

}

