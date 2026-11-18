"""Shared world-to-pixel geometry and standalone full-session semantic maps."""
import json
import math
import os
import re
import shutil
from PIL import Image, ImageDraw, ImageFont
from session_catalog import STAMP_PREFIX, session_label
from viewer_core import frame_metadata
from semantic_replay import replay_geometry


class Viewport:
    def __init__(self, centre, span, width, height):
        self.centre=tuple(centre);self.span=float(span)
        self.width=max(1,int(width));self.height=max(1,int(height))
        self.scale=self.width/self.span
        self.left=self.centre[0]-self.span/2
        self.top=self.centre[1]+self.height/self.scale/2

    @classmethod
    def fit(cls,bounds,width,height,margin=1.):
        x0,y0,x1,y1=bounds
        span=max(x1-x0+2*margin,(y1-y0+2*margin)*width/max(1.,height),2.)
        return cls(((x0+x1)/2,(y0+y1)/2),span,width,height)

    def pixel(self,x,y,clip=True):
        px,py=(x-self.left)*self.scale,(self.top-y)*self.scale
        return (px,py) if not clip or (0<=px<self.width and 0<=py<self.height) else None


def point_bounds(points):
    pts=list(points)
    if not pts:return (-2.,-2.,2.,2.)
    return min(p[0] for p in pts),min(p[1] for p in pts),max(p[0] for p in pts),max(p[1] for p in pts)


def font(size):
    for path in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','C:/Windows/Fonts/arial.ttf'):
        if os.path.isfile(path):return ImageFont.truetype(path,size)
    return ImageFont.load_default()


def text_size(draw,text,face):
    try:
        box=draw.textbbox((0,0),text,font=face);return box[2]-box[0]+6,box[3]+4
    except (AttributeError,ValueError):
        w,h=draw.textsize(text,font=face);return w+6,h+4


def load_grid(info):
    with Image.open(info['image']) as image:return image.convert('RGB')


def grid_bounds(info,image):
    grey=image.convert('L')
    box=grey.point(lambda v:0 if v==205 else 255).getbbox() or (0,0,image.width,image.height)
    ox,oy,yaw=info['origin'];res=float(info['resolution']);c,s=math.cos(yaw),math.sin(yaw)
    points=[]
    for u,v in ((box[0],box[1]),(box[0],box[3]),(box[2],box[1]),(box[2],box[3])):
        x,y=u*res,(image.height-v)*res;points.append((ox+c*x-s*y,oy+s*x+c*y))
    return point_bounds(points)


def raster_view(image,info,view):
    """Output pixel -> map raster; honours resolution, row direction and yaw."""
    ox,oy,yaw=info['origin'];res=float(info['resolution']);c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=view.left-ox,view.top-oy;k=1./(view.scale*res)
    affine=(c*k,-s*k,(c*dx+s*dy)/res,
            s*k,c*k,image.height-(-s*dx+c*dy)/res)
    return image.transform((view.width,view.height),Image.AFFINE,affine,resample=Image.NEAREST)


def draw_objects(image,view,objects,active=(),size=12):
    draw=ImageDraw.Draw(image);face=font(size);occupied=[]
    for i,obj in enumerate(objects):
        pos=view.pixel(obj['x'],obj['y'])
        if pos is None:continue
        x,y=pos;number=obj.get('number',i+1);color=(74,222,128) if number in active else (30,155,220)
        radius=max(3,size//3)
        draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=color,outline=(10,25,35))
        text='#%d %s'%(number,obj['label']);w,h=text_size(draw,text,face)
        options=[]
        for dx,dy in ((8,-h),(8,5),(-w-8,-h),(-w-8,5),(8,-2*h-4),(8,h+8)):
            tx=max(0,min(view.width-w,x+dx));ty=max(0,min(view.height-h,y+dy))
            box=(tx,ty,tx+w,ty+h)
            overlaps=sum(max(0,min(box[2],r[2])-max(box[0],r[0]))*max(0,min(box[3],r[3])-max(box[1],r[1])) for r in occupied)
            options.append((overlaps,box))
        _,box=min(options,key=lambda item:item[0])
        occupied.append(box)
        draw.line((x,y,box[0]+w/2,box[1]+h/2),fill=color,width=1)
        draw.rectangle(box,fill=(13,17,23))
        draw.text((box[0]+3,box[1]),text,font=face,fill=color)


def gallery_filename(run,folder):
    name=run.get('run_id') or os.path.basename(folder)
    return (re.sub('[^A-Za-z0-9_-]+','_',name).strip('_') or 'semantic_map')+'.png'


def gallery_recording(source_path):
    """Directory key for one recording, including legacy bags in shared folders."""
    parent, filename = os.path.split(source_path.replace('\\', '/'))
    stem = os.path.splitext(filename)[0]
    stem = re.sub(r'(_\d{8}_\d{6})_\d+$', r'\1', stem)
    recording = session_label(stem, limit=100) or 'recording'
    session = os.path.basename(parent)
    if STAMP_PREFIX.match(session):
        return session_label(session, limit=100) + '__' + recording
    return recording


def gallery_image_path(run, folder):
    sources = run.get('sources', [])
    recording = gallery_recording(sources[0]['path']) if sources else 'unassigned'
    return os.path.join(recording, gallery_filename(run, folder))


def export_final_map(folder,destination=None,size=(2400,1600),gallery=None):
    """Uses saved run geometry, objects and sample poses; no ROS/model/bag required."""
    folder=os.path.abspath(folder)
    with open(os.path.join(folder,'run.json')) as f:run=json.load(f)
    if run.get('state')!='complete':raise ValueError('Semantic run is not complete')
    with open(os.path.join(folder,'objects.json')) as f:data=json.load(f)
    world=run['world_frame']
    if data['world_frame']!=world:raise ValueError('Object and map coordinate frames differ')
    objects=data['objects'];path=[]
    for meta_path in frame_metadata(folder):
        with open(meta_path) as f:meta=json.load(f)
        if meta.get('world_frame')!=world:raise ValueError('Mixed trajectory coordinate frames')
        m=meta.get('t_world_optical',meta.get('t_map_cam'))
        if m:path.append((m[0][3],m[1][3]))
    points=path+[(o['x'],o['y']) for o in objects]
    width,height=size;head,foot=94,76
    view=Viewport.fit(point_bounds(points),width-40,height-head-foot,margin=1.25)
    info=replay_geometry(run,folder) if world=='map' else None
    if world=='map' and not info:raise ValueError('Map geometry is missing')
    if data.get('geometry',{}).get('map')!=run.get('geometry',{}).get('map'):
        raise ValueError('Object and background map provenance differ')
    if info:
        import yaml
        with open(info['yaml']) as f:grid=yaml.safe_load(f)
        image_path=grid['image']
        if not os.path.isabs(image_path):image_path=os.path.join(os.path.dirname(info['yaml']),image_path)
        grid['image']=image_path
        panel=raster_view(load_grid(grid),grid,view)
    else:panel=Image.new('RGB',(view.width,view.height),(235,239,243))
    draw=ImageDraw.Draw(panel)
    if len(path)>1:draw.line([view.pixel(x,y,False) for x,y in path],fill=(110,115,125),width=3)
    draw_objects(panel,view,objects,size=22)
    result=Image.new('RGB',size,(248,250,252));result.paste(panel,(20,head))
    draw=ImageDraw.Draw(result)
    title='Full-session semantic candidates | '+run.get('run_id',os.path.basename(folder))
    draw.text((24,12),title,font=font(27),fill=(20,35,50))
    counts={}
    for obj in objects:counts[obj['label']]=counts.get(obj['label'],0)+1
    description='%s | %s | grey: camera trajectory'%(world.upper(),', '.join('%s: %d'%(k,v) for k,v in sorted(counts.items())) or 'no fused candidates')
    if world=='odom':description+=' | ODOM ONLY - no occupancy map'
    draw.text((24,53),description,font=font(23),fill=(75,85,95))
    draw.text((24,height-64),'Numbers match this run in the viewer. Points are visible-surface estimates, not verified object centres.',font=font(21),fill=(75,85,95))
    draw.text((24,height-34),'Full-session fusion includes later observations. More candidates do not imply better accuracy.',font=font(21),fill=(75,85,95))
    destination=destination or os.path.join(folder,'final_semantic_map.png')
    result.save(destination+'.tmp',format='PNG');os.replace(destination+'.tmp',destination)
    if gallery:
        gallery_image = os.path.join(gallery, gallery_image_path(run, folder))
        os.makedirs(os.path.dirname(gallery_image), exist_ok=True)
        shutil.copy2(destination, gallery_image)
    return os.path.abspath(destination)


def export_recording_map(folder, gallery):
    path = export_final_map(folder)
    with open(os.path.join(folder, 'run.json')) as f:
        run = json.load(f)
    os.makedirs(gallery, exist_ok=True)
    return shutil.copy2(path, os.path.join(gallery, gallery_filename(run, folder)))
