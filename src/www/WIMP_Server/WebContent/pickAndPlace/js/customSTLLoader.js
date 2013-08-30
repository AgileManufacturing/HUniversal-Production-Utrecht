
	
	var orgX,orgY;

	var container;
	var move=false;
	var wire=false;
	
    function readAsciiStl(l, vs, fs)
    {
    	var solid=false;
		var face=false;
		var vis=[];
		vtest={};
        
		for(var i=0;i<l.length;i++)
		{
			var line=l[i];
			if(solid)
			{
				if(line.search("endsolid")>-1)
					solid=false;
				else
				if(face)
				{
					if(line.search("endfacet")>-1)
					{
						face=false;
						var f = new THREE.Face3(vis[0], vis[1], vis[2]);
						fs.push(f);
					}
					else
					if(line.search("vertex")>-1)
					{
						var cs = line.substr(line.search("vertex")+7);
						cs=cs.trim();
						var ls=cs.split(/\s+/);
						var vv = new THREE.Vector3(parseFloat(ls[0]),parseFloat(ls[1]),parseFloat(ls[2]));
						var v = new THREE.Vertex();v.position=vv;
						var vi=vs.length;
						if(cs in vtest)
						{
							vi=vtest[cs];
						}
						else
						{
							vs.push(v)
							vtest[cs]=vi;
						}
						vis.push(vi);
					}
				}
				else
				{
					if(line.search("facet normal")>-1)
					{
						face=true;
						vis=[];
					}
				}
			}
			else
			{
				if(line.search("solid")>-1)
					solid=true;
			}
		}
		vtest=null;

    }
    
    function triangle(){
         if(arguments.length==2){ 
             this._buffer=arguments[0]; 
             this._sa = arguments[1]; 
        }else{ 
             this._sa =0; 
            this._buffer = new ArrayBuffer(50);}
        this.__byte = new Uint8Array(this._buffer);
        this.normal = new Float32Array(this._buffer, this._sa+0, 3);
        this.v1 = new Float32Array(this._buffer, this._sa+12, 3);
        this.v2 = new Float32Array(this._buffer, this._sa+24, 3);
        this.v3 = new Float32Array(this._buffer, this._sa+36, 3);
        var _attr = new Int16Array(this._buffer, this._sa+48, 1);
            Object.defineProperty(this, "attr",{get:function(){
                return _attr[0];},set: function(val){
            _attr[0] = val;    },enumerable  : true});    
    };
    
    function readBinaryStl(l, vs, fs)
    {
        var buf = new ArrayBuffer(l.length);
        var bbuf= new Uint8Array(buf);
        for(var i=0;i<l.length;i++)
            bbuf[i]=l.charCodeAt(i);
        var trnr=new Uint32Array(buf,80,1);
        var vis=[0,0,0];
		vtest={};
        var offset=84;
        var face=new triangle();
        for(var i=0;i<trnr[0];i++)
        {
            for(var j=0;j<50;j++)
                face.__byte[j]=bbuf[offset+j];
            
            var vv = new THREE.Vector3(face.v1[0], face.v1[1], face.v1[2]);
			var v = new THREE.Vertex();v.position=vv;
            var k=""+face.v1[0]+","+face.v1[1]+","+face.v1[2];
            vis[0]=vs.length;
            if(k in vtest)
                vis[0]=vtest[k];
            else
            {
                vs.push(v)
                vtest[k]=vis[0];
            }

            vv = new THREE.Vector3(face.v2[0], face.v2[1], face.v2[2]);
			v = new THREE.Vertex();v.position=vv;
            k=""+face.v2[0]+","+face.v2[1]+","+face.v2[2];
            vis[1]=vs.length;
            if(k in vtest)
                vis[1]=vtest[k];
            else
            {
                vs.push(v)
                vtest[k]=vis[1];
            }

            
            vv = new THREE.Vector3(face.v3[0], face.v3[1], face.v3[2]);
			v = new THREE.Vertex();v.position=vv;
            k=""+face.v3[0]+","+face.v3[1]+","+face.v3[2];
            vis[2]=vs.length;
            if(k in vtest)
                vis[2]=vtest[k];
            else
            {
                vs.push(v)
                vtest[k]=vis[2];
            }

            var normal = new THREE.Vector3( face.normal[0], face.normal[1], face.normal[2] );
            var f = new THREE.Face3(vis[0], vis[1], vis[2], normal);
			fs.push(f);
            
            offset+=50;
        }
        vtest = null;
        delete bbuf;
        delete buf;
        buf=null;
    }
    
    
	function readStl(oFile, vs, fs)
	{
        var solididx = oFile.search("solid");
        if(solididx>-1 && solididx<10)
        {
            var l = oFile.split(/[\r\n]/g);
            readAsciiStl(l, vs, fs);
        }
        else
        {
            readBinaryStl(oFile, vs, fs);
        }
	}
	
	function buildGeometry(l, f)
	{
		var vs=[];
		var fs=[];
		if(f.name.search(".obj")>-1)
			readObj(l, vs, fs);
			else
		if(f.name.search(".stl")>-1)
			readStl(l, vs, fs);
			else
			return;
		for(var i in fs)
		{
			var v0=vs[fs[i].a].position;
			var v1=vs[fs[i].b].position;
			var v2=vs[fs[i].c].position;
			var e1=new THREE.Vector3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
			var e2=new THREE.Vector3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
			var n=new THREE.Vector3(e1.y*e2.z-e1.z*e2.y, e1.z*e2.x-e1.x*e2.z, e1.x*e2.y-e1.y*e2.x);
			var l = Math.sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
			n.x/=l;n.y/=l;n.z/=l;
			fs[i].normal=n;
		}
		var mx=1e10,my=1e10,mz=1e10;
		var Mx=-1e10,My=-1e10,Mz=-1e10;
		for(var i in vs)
		{
			if(mx>vs[i].position.x)mx=vs[i].position.x;
			if(my>vs[i].position.y)my=vs[i].position.y;
			if(mz>vs[i].position.z)mz=vs[i].position.z;
			if(Mx<vs[i].position.x)Mx=vs[i].position.x;
			if(My<vs[i].position.y)My=vs[i].position.y;
			if(Mz<vs[i].position.z)Mz=vs[i].position.z;
		}
		var max = Math.max(Mx-mx,My-my,Mz-mz);
		max/=200;
		var cx=(Mx+mx)/2;
		var cy=(My+my)/2;
		var cz=(Mz+mz)/2;
		for(var i in vs)
		{
			vs[i].position.x-=cx;
			vs[i].position.y-=cy;
			vs[i].position.z-=cz;
			vs[i].position.x/=max;
			vs[i].position.y/=max;
			vs[i].position.z/=max;
		}
		var mx=1e10,my=1e10,mz=1e10;
		var Mx=-1e10,My=-1e10,Mz=-1e10;
		for(var i in vs)
		{
			if(mx>vs[i].position.x)mx=vs[i].position.x;
			if(my>vs[i].position.y)my=vs[i].position.y;
			if(mz>vs[i].position.z)mz=vs[i].position.z;
			if(Mx<vs[i].position.x)Mx=vs[i].position.x;
			if(My<vs[i].position.y)My=vs[i].position.y;
			if(Mz<vs[i].position.z)Mz=vs[i].position.z;
		}
		geometry = new THREE.Geometry();
		geometry.vertices = vs;
		geometry.faces = fs;
		if(mesh)
			scene.remove( mesh );
		mesh = new THREE.Mesh( geometry, material );
        scene.add( mesh );
		render();
	}
	
	function handleFiles(f)
    {
		document.getElementById("centered").style.display = 'none';
        var reader = new FileReader();
        reader.onload=function(e){
            var oFile=e.target.result;
			buildGeometry(oFile, f[0]);
        };
        var file = f[0];
		reader.readAsBinaryString(file);
    }
	
	function example(file)
	{
		var xhr = new XMLHttpRequest();
		xhr.open('GET',  file, true);
		xhr.onload  = function(){
			document.getElementById("centered").style.display = 'none';
			var oFile=this.response;
			f={};
			f.name=file;
			buildGeometry(oFile, f);
	  }
		xhr.send("");
	}
	
	function load()
	{
		init();
		if (window.File && window.FileReader && window.FileList && window.Blob) {
		// Great success! All the File APIs are supported.
		} else {
		  alert("The File API is needed for this application! Your browser is not supported!");
		}
	}
	