#include <chrono>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <random>
#include <cstdint>
#include <cstring>
#include <future>
#include <vector>
#include "threads_pool.hpp"
using namespace std;

constexpr size_t N=36;
constexpr size_t M=84;
constexpr size_t T=1000000;
constexpr array<pair<int,int>,4> deltas{{{-1,0},{1,0},{0,-1},{0,1}}};
char field[N][M+1]={
"####################################################################################",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                       .........                                  #",
"#..............#            #           .........                                  #",
"#..............#            #           .........                                  #",
"#..............#            #           .........                                  #",
"#..............#            #                                                      #",
"#..............#            #                                                      #",
"#..............#            #                                                      #",
"#..............#            #                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............#                                                      #",
"#..............#............################                     #                 #",
"#...........................#....................................#                 #",
"#...........................#....................................#                 #",
"#...........................#....................................#                 #",
"##################################################################                 #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"#                                                                                  #",
"####################################################################################",
};
struct Fixed{constexpr Fixed(int v):v(v<<16){}constexpr Fixed(float f):v((int32_t)(f*(1<<16))){}constexpr Fixed(double f):v((int32_t)(f*(1<<16))){}constexpr Fixed():v(0){}static constexpr Fixed from_raw(int32_t x){Fixed r;r.v=x;return r;}int32_t v;auto operator<=>(const Fixed&)const=default;bool operator==(const Fixed&)const=default;};
static constexpr Fixed inf=Fixed::from_raw(numeric_limits<int32_t>::max());
static constexpr Fixed eps=Fixed::from_raw((int32_t)deltas.size());
inline Fixed operator+(Fixed a,Fixed b){return Fixed::from_raw(a.v+b.v);}
inline Fixed operator-(Fixed a,Fixed b){return Fixed::from_raw(a.v-b.v);}
inline Fixed operator*(Fixed a,Fixed b){return Fixed::from_raw((int64_t)a.v*b.v>>16);}
inline Fixed operator/(Fixed a,Fixed b){return Fixed::from_raw(((int64_t)a.v<<16)/b.v);}
inline Fixed& operator+=(Fixed &a,Fixed b){a=Fixed::from_raw(a.v+b.v);return a;}
inline Fixed& operator-=(Fixed &a,Fixed b){a=Fixed::from_raw(a.v-b.v);return a;}
inline Fixed& operator*=(Fixed &a,Fixed b){a=Fixed::from_raw(((int64_t)a.v*b.v)>>16);return a;}
inline Fixed& operator/=(Fixed &a,Fixed b){a=Fixed::from_raw(((int64_t)a.v<<16)/b.v);return a;}
inline Fixed operator-(Fixed x){return Fixed::from_raw(-x.v);}
inline Fixed absf(Fixed x){if(x.v<0)x.v=-x.v;return x;}
inline ostream& operator<<(ostream &o,Fixed x){return o<<x.v/(double)(1<<16);}
Fixed rho[256];
Fixed p[N][M],old_p[N][M];
struct VectorField{array<Fixed,deltas.size()>v[N][M];inline Fixed&add(int x,int y,int dx,int dy,Fixed dv){return get(x,y,dx,dy)+=dv;}inline Fixed&get(int x,int y,int dx,int dy){size_t i=find(deltas.begin(),deltas.end(),pair(dx,dy))-deltas.begin();return v[x][y][i];}};
VectorField velocity,velocity_flow;
int last_use[N][M];
int UT=0;
mt19937 rnd(1337);
inline Fixed random01(){return Fixed::from_raw(rnd()&((1<<16)-1));}
int dirs[N][M];
tuple<Fixed,bool,pair<int,int>> propagate_flow(int x,int y,Fixed lim){last_use[x][y]=UT-1;Fixed r=Fixed::from_raw(0);for(auto &[dx,dy]:deltas){int nx=x+dx,ny=y+dy;if(field[nx][ny]!='#'&&last_use[nx][ny]<UT){auto cap=velocity.get(x,y,dx,dy);auto fl=velocity_flow.get(x,y,dx,dy);if(fl==cap)continue;auto vp=(cap-fl<lim?cap-fl:lim);if(last_use[nx][ny]==UT-1){velocity_flow.add(x,y,dx,dy,vp);last_use[x][y]=UT;return{vp,true,{nx,ny}};}auto[t,prop,end]=propagate_flow(nx,ny,vp);r+=t;if(prop){velocity_flow.add(x,y,dx,dy,t);last_use[x][y]=UT;return{t,prop&&end!=pair(x,y),end};}}}last_use[x][y]=UT;return{r,false,{0,0}};}
void propagate_stop(int x,int y,bool f=false){if(!f){bool s=true;for(auto &[dx,dy]:deltas){int nx=x+dx,ny=y+dy;if(field[nx][ny]!='#'&&last_use[nx][ny]<UT-1&&velocity.get(x,y,dx,dy)>Fixed::from_raw(0)){s=false;break;}}if(!s)return;}last_use[x][y]=UT;for(auto &[dx,dy]:deltas){int nx=x+dx,ny=y+dy;if(field[nx][ny]=='#'||last_use[nx][ny]==UT||velocity.get(x,y,dx,dy)>Fixed::from_raw(0))continue;propagate_stop(nx,ny);}}
inline Fixed move_prob(int x,int y){Fixed s=Fixed::from_raw(0);for(size_t i=0;i<deltas.size();++i){auto &[dx,dy]=deltas[i];int nx=x+dx,ny=y+dy;if(field[nx][ny]=='#'||last_use[nx][ny]==UT)continue;auto vv=velocity.get(x,y,dx,dy);if(vv<Fixed::from_raw(0))continue;s+=vv;}return s;}
struct ParticleParams{char c;Fixed cp;array<Fixed,deltas.size()>v;inline void swap_with(int x,int y){std::swap(field[x][y],c);std::swap(p[x][y],cp);std::swap(velocity.v[x][y],v);}};
bool propagate_move(int x,int y,bool isf){last_use[x][y]=UT-(int)isf;bool r=false;int nx=-1,ny=-1;do{array<Fixed,deltas.size()>tres;Fixed s=Fixed::from_raw(0);for(size_t i=0;i<deltas.size();++i){auto [dx,dy]=deltas[i];int xx=x+dx,yy=y+dy;if(field[xx][yy]=='#'||last_use[xx][yy]==UT){tres[i]=s;continue;}auto vv=velocity.get(x,y,dx,dy);if(vv<Fixed::from_raw(0)){tres[i]=s;continue;}s+=vv;tres[i]=s;}if(s==Fixed::from_raw(0))break;Fixed p0=random01()*s;size_t d=size_t(upper_bound(tres.begin(),tres.end(),p0)-tres.begin());auto[dx,dy]=deltas[d];nx=x+dx;ny=y+dy;r=(last_use[nx][ny]==UT-1||propagate_move(nx,ny,false));}while(!r);last_use[x][y]=UT;for(size_t i=0;i<deltas.size();++i){auto [dx,dy]=deltas[i];int nx2=x+dx,ny2=y+dy;if(field[nx2][ny2]!='#'&&last_use[nx2][ny2]<UT-1&&velocity.get(x,y,dx,dy)<Fixed::from_raw(0))propagate_stop(nx2,ny2);}if(r&&!isf){ParticleParams pp{};pp.swap_with(x,y);pp.swap_with(nx,ny);pp.swap_with(x,y);}return r;}
ThreadPool* gThreadPool=nullptr;

inline void parallel_rows(size_t numThreads,function<void(size_t,size_t)>f){
    size_t chunk=(N+numThreads-1)/numThreads;
    atomic<size_t>cnt(0);
    vector<future<void>>fs;
    fs.reserve(numThreads);
    for(size_t t=0;t<numThreads;++t){
        fs.push_back(async(launch::async,[&,t](){
            size_t start=t*chunk,end=(start+chunk<N?start+chunk:N);
            for(size_t i=start;i<end;++i)f(i,end);
            cnt.fetch_add(1,memory_order_relaxed);
        }));
    }
    for(auto &ff:fs)ff.get();
}

int main(int argc,char**argv){
    ios::sync_with_stdio(false);cin.tie(nullptr);
    size_t numThreads=4;if(argc>1)numThreads=stoul(argv[1]);
    ThreadPool pool(numThreads);gThreadPool=&pool;
    rho[' ']=Fixed(0.01f);rho['.']=Fixed(1000.f);
    for(size_t x=0;x<N;++x){
        for(size_t y=0;y<M;++y){
            if(field[x][y]=='#')continue;
            for(auto &[dx,dy]:deltas){
                dirs[x][y]+=(field[x+dx][y+dy]!='#');
            }
        }
    }
    auto st=chrono::high_resolution_clock::now();
    for(size_t i=0;i<T;++i){
        {
            size_t chunk=(N+numThreads-1)/numThreads;
            vector<future<void>>fs;
            fs.reserve(numThreads);
            for(size_t t=0;t<numThreads;++t){
                fs.push_back(async(launch::async,[&,t](){
                    size_t start=t*chunk;size_t end=(start+chunk<N?start+chunk:N);
                    for(size_t x=start;x<end;++x){
                        for(size_t y=0;y<M;++y){
                            if(field[x][y]=='#')continue;
                            if(field[x+1][y]!='#')velocity.add(x,y,1,0,Fixed(0.1f));
                        }
                    }
                }));
            }
            for(auto &f:fs)f.get();
        }
        {
            size_t chunk=(N+numThreads-1)/numThreads;
            vector<future<void>>fs;
            fs.reserve(numThreads);
            for(size_t t=0;t<numThreads;++t){
                fs.push_back(async(launch::async,[&,t](){
                    size_t start=t*chunk;size_t end=(start+chunk<N?start+chunk:N);
                    for(size_t x=start;x<end;++x){
                        memcpy(old_p[x],p[x],sizeof(p[x]));
                    }
                }));
            }
            for(auto &f:fs)f.get();
        }
        {
            size_t chunk=(N+numThreads-1)/numThreads;
            vector<future<void>>fs;
            fs.reserve(numThreads);
            for(size_t t=0;t<numThreads;++t){
                fs.push_back(async(launch::async,[&,t](){
                    size_t start=t*chunk;size_t end=(start+chunk<N?start+chunk:N);
                    for(size_t x=start;x<end;++x){
                        for(size_t y=0;y<M;++y){
                            if(field[x][y]=='#')continue;
                            for(auto &[dx,dy]:deltas){
                                int nx=x+dx,ny=y+dy;
                                if(field[nx][ny]!='#'&&old_p[nx][ny]<old_p[x][y]){
                                    Fixed dp=old_p[x][y]-old_p[nx][ny];
                                    Fixed force=dp;
                                    auto &ctr=velocity.get(nx,ny,-dx,-dy);
                                    if(ctr*rho[(int)field[nx][ny]]>=force){
                                        ctr-=force/rho[(int)field[nx][ny]];
                                        continue;
                                    }
                                    force-=ctr*rho[(int)field[nx][ny]];
                                    ctr=Fixed::from_raw(0);
                                    velocity.add(x,y,dx,dy,force/rho[(int)field[x][y]]);
                                    p[x][y]-=force/Fixed((int)dirs[x][y]);
                                }
                            }
                        }
                    }
                }));
            }
            for(auto &f:fs)f.get();
        }
        velocity_flow=VectorField();
        bool prop=false;
        do{
            UT+=2;prop=false;
            for(size_t x=0;x<N;++x){
                for(size_t y=0;y<M;++y){
                    if(field[x][y]!='#'&&last_use[x][y]!=UT){
                        auto[t,pp,_]=propagate_flow(x,y,Fixed::from_raw(1));
                        if(t>Fixed::from_raw(0))prop=true;
                    }
                }
            }
        }while(prop);
        {
            size_t chunk=(N+numThreads-1)/numThreads;
            vector<future<void>>fs;
            fs.reserve(numThreads);
            for(size_t t=0;t<numThreads;++t){
                fs.push_back(async(launch::async,[&,t](){
                    size_t start=t*chunk;size_t end=(start+chunk<N?start+chunk:N);
                    for(size_t x=start;x<end;++x){
                        for(size_t y=0;y<M;++y){
                            if(field[x][y]=='#')continue;
                            for(auto &[dx,dy]:deltas){
                                auto ov=velocity.get(x,y,dx,dy);
                                auto nv=velocity_flow.get(x,y,dx,dy);
                                if(ov>Fixed::from_raw(0)){
                                    auto ff=(ov-nv)*rho[(int)field[x][y]];
                                    if(field[x][y]=='.')ff*=Fixed(0.8f);
                                    if(field[x+dx][y+dy]=='#'){
                                        p[x][y]+=ff/Fixed((int)dirs[x][y]);
                                    }else{
                                        p[x+dx][y+dy]+=ff/Fixed((int)dirs[x+dx][y+dy]);
                                    }
                                    velocity.get(x,y,dx,dy)=nv;
                                }
                            }
                        }
                    }
                }));
            }
            for(auto &f:fs)f.get();
        }
        UT+=2;bool anyp=false;
        for(size_t x=0;x<N;++x){
            for(size_t y=0;y<M;++y){
                if(field[x][y]=='#'||last_use[x][y]==UT)continue;
                if(random01()<move_prob(x,y)){
                    anyp=true;propagate_move(x,y,true);
                }else{
                    propagate_stop(x,y,true);
                }
            }
        }
        if(anyp){
            cout<<"Tick "<<i<<":\n";
            for(size_t x=0;x<N;++x)cout<<field[x]<<'\n';
        }
    }
    auto en=chrono::high_resolution_clock::now();
    cout<<(chrono::duration<double>(en-st).count())<<"\n";
    return 0;
}