"""Local result intake only. Never launches ROS, Gazebo, board or network operations."""
import argparse, csv, json, shutil, datetime, html
from pathlib import Path

P=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--intake',type=Path)
args=parser.parse_args()
rows=json.loads((P/'metrics/results.json').read_text(encoding='utf-8'))
canonical_status={'perception':'FAIL','recognition_accuracy':'NOT_MEASURED'}
for result_id,status in canonical_status.items():
    matches=[r for r in rows if r['id']==result_id]
    if len(matches)!=1 or matches[0]['status']!=status:
        raise ValueError(f'Canonical status mismatch for {result_id}: expected {status}')
recognition=next(r for r in rows if r['id']=='recognition_accuracy')
if '未达到' in recognition['note']:
    raise ValueError('Official recognition status must stay NOT_MEASURED without an unmet-result claim')
perception=next(r for r in rows if r['id']=='perception')
if ('0/0/76' in perception['note'] or 'policy 0/8/76' in perception['note']) and '撤回' not in perception['note']:
    raise ValueError('Withdrawn perception policy metrics must not be reported as measured')
if 'NOT_RUN' not in perception['note'] or 'NOT_MEASURED' not in perception['note']:
    raise ValueError('Perception policy must remain explicitly NOT_RUN/NOT_MEASURED')
braking=next(r for r in rows if r['id']=='braking_s')
if '1.0s' not in braking['note'] or '1.5s' not in braking['note']:
    raise ValueError('Braking result must distinguish the predeclared 1.0 s hold from the observed 1.5 s hold')
allowed_statuses=['PASS','FAIL','PARTIAL','NOT_MEASURED']
allowed_bases={
    'measured_simulation',
    'internal_frozen_regression',
    'offline_replay',
    'synthetic_replay',
    'design_calculation',
    'not_measured',
}
if args.intake:
    incoming=json.loads(args.intake.read_text(encoding='utf-8-sig'))
    known={r['id']:r for r in rows}
    planned=[]
    for update in incoming:
        if update['id'] not in known or update['status'] not in allowed_statuses:
            raise ValueError('Unknown id or invalid status')
        if update['status']!='NOT_MEASURED' and (update.get('basis') not in allowed_bases or not update.get('evidence')):
            raise ValueError('Measured or bounded intake requires a declared evidence basis and actual evidence')
        if not update.get('note') or not update.get('run_id') or not update.get('source_commit'):
            raise ValueError('Need note, run_id, source_commit')
        evidence=[Path(x).resolve(strict=True) for x in update.get('evidence',[])]
        if any(not f.is_file() for f in evidence): raise ValueError('Evidence must be files')
        if update['id'] in ['localization_mm','mapped_area_m2','cleaning_m2_h','width_mm','braking_s'] and update['status']!='NOT_MEASURED':
            val=update.get('value')
            if not isinstance(val,(int,float)) or isinstance(val,bool) or not __import__('math').isfinite(val) or val<0: raise ValueError('Need finite nonnegative measured value in declared units')
            if update['status'] in ['PASS','FAIL']:
                threshold={'localization_mm':50,'mapped_area_m2':20000,'cleaning_m2_h':3500,'width_mm':600,'braking_s':1}[update['id']]
                passed=val<=threshold if update['id'] in ['localization_mm','braking_s'] else val>=threshold
                if (update['status']=='PASS')!=passed: raise ValueError('Status contradicts threshold')
        planned.append((update,evidence))
    stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    dest=P/'evidence/intake'/stamp
    dest.mkdir(parents=True)
    shutil.copyfile(P/'metrics/results.json',dest/'previous-results.json')
    for update,files in planned:
        local=[]
        for i,f in enumerate(files):
            target=dest/update['id']/f'{i:02d}-{f.name}'; target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(f,target); local.append(target.relative_to(P).as_posix())
        target=known[update['id']]
        for k in ['status','value','basis','note','run_id','source_commit','rollback_commit']:
            target[k]=update.get(k)
        target['evidence']=local
        target['as_of_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    (P/'metrics/results.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')

for r in rows:
    assert r['status'] in allowed_statuses
    if r['status']!='NOT_MEASURED':
        assert r['basis'] in allowed_bases and r['evidence']
    if r['status'] in ['PASS','FAIL','PARTIAL']:
        assert r.get('run_id') and r.get('source_commit') and r.get('evidence')
    if r['id'] in ['localization_mm','mapped_area_m2','cleaning_m2_h','width_mm','braking_s'] and r['status']!='NOT_MEASURED':
        val=r.get('value')
        assert isinstance(val,(int,float)) and not isinstance(val,bool) and __import__('math').isfinite(val) and val>=0
        if r['status'] in ['PASS','FAIL']:
            threshold={'localization_mm':50,'mapped_area_m2':20000,'cleaning_m2_h':3500,'width_mm':600,'braking_s':1}[r['id']]
            passed=val<=threshold if r['id'] in ['localization_mm','braking_s'] else val>=threshold
            assert (r['status']=='PASS')==passed
    for path in r['evidence']: assert (P/path).is_file(),path
with (P/'metrics/sim-metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.DictWriter(f,fieldnames=['id','item','status','value','basis','note','evidence','as_of_utc'],extrasaction='ignore')
    writer.writeheader(); writer.writerows(rows)
md=['# 唯一最小验收矩阵','', '状态仅适用于列明证据范围。报告是仿真路线，不证明实车；未测与已测失败分开。','', '|条目|状态|值/范围|证据|','|---|---|---|---|']
for r in rows:
    links='、'.join(f'[{Path(x).name}](../{x})' for x in r['evidence'])
    md.append(f"|{r['item']}|{r['status']}|{r['note']}|{links}|")
(P/'metrics/验收矩阵.md').write_text('\n'.join(md),encoding='utf-8')

from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
pdfmetrics.registerFont(TTFont('CN',r'C:\Windows\Fonts\simsun.ttc',subfontIndex=0))
sections=json.loads((P/'docs/sections.json').read_text(encoding='utf-8'))
if len(sections)!=len(rows)+9:
    raise ValueError(f'Expected {len(rows)+9} report sections, found {len(sections)}')
for i,r in enumerate(rows):
    sections[i+2][0]=f'{i+3:02d} {r["item"]}'
    sections[i+2][1]=['当前状态：'+r['status']+'。'+r['note'], '最短测量与判定口径：'+r['minimum_measurement'], '证据范围：'+r['basis']+'；截至 '+r['as_of_utc']+'。只依据列明运行判定，历史局部结果不替代当前全程。', '证据文件：'+'；'.join(r['evidence'])]
(P/'docs/sections.json').write_text(json.dumps(sections,ensure_ascii=False,indent=2),encoding='utf-8')
total_pages=len(sections)
style=ParagraphStyle('body',fontName='CN',fontSize=12,leading=23,wordWrap='CJK',textColor='#263849')
c=canvas.Canvas(str(P/'docs/技术方案报告.pdf'),pagesize=(595.28,841.89))
c.setTitle('DG-202604 智慧环卫无人清扫车技术方案报告')
for page,(title,paras) in enumerate(sections,1):
    c.setFillColorRGB(.06,.18,.26); c.rect(0,745,595.28,96,fill=1,stroke=0)
    c.setFillColorRGB(1,1,1); c.setFont('CN',19); c.drawString(43,790,title)
    c.setFont('CN',10); c.drawString(43,762,'DG-202604 | 独立仿真与S100P回放 | 证据边界明确披露')
    y=713
    for para in paras:
        p=Paragraph(html.escape(para),style); w,h=p.wrap(505,650)
        if y-h<100: raise ValueError(f'Page {page} overflow')
        p.drawOn(c,45,y-h); y-=h+24
    # Measurement design diagrams are explicitly labelled; no synthetic outcome plots.
    if 3<=page<=2+len(rows):
        c.setFillColorRGB(.93,.96,.98); c.roundRect(45,122,505,205,7,fill=1,stroke=0)
        c.setFillColorRGB(.06,.18,.26); c.setFont('CN',12)
        c.drawString(62,301,'测量设计图（不是运行结果）')
        labels=['原始运行数据','独立计算/判据','报告与演示对照']
        for k,label in enumerate(labels):
            x=62+k*163
            c.setStrokeColorRGB(.15,.4,.5); c.roundRect(x,220,150,43,4,fill=0,stroke=1)
            c.setFont('CN',11); c.drawString(x+10,237,label)
            if k<2: c.line(x+150,241,x+163,241)
        c.setFont('CN',10)
        c.drawString(62,181,'记录同一次任务、配置与时钟；排除未知区域、重复面积或陈旧输入。')
        c.drawString(62,155,'没有对应实测时保留NOT_MEASURED；估算只用于安排最短补测。')
    if page==17:
        c.setFillColorRGB(.06,.18,.26); c.setFont('CN',12)
        c.drawString(45,300,'板端规划回放：成功次数 / 各策略3次（历史实测报告）')
        for k,(label,n) in enumerate([('full_coverage',3),('sensing_greedy',2),('oracle / evaluation_only',1)]):
            yy=260-k*47
            c.setFont('CN',10); c.drawString(45,yy+8,label)
            c.setFillColorRGB(.12,.47,.54); c.rect(210,yy,n*80,23,fill=1,stroke=0)
            c.setFillColorRGB(.06,.18,.26); c.drawString(460,yy+7,f'{n}/3')
        c.drawString(45,110,'输入为simulation replay；不代表当前Gazebo完整任务成功率。')
    if page==2:
        for k,label in enumerate(['传感器输入','算法实际计算','控制输出','环境/独立评分']):
            x=45+k*128; c.setStrokeColorRGB(.1,.4,.5); c.roundRect(x,145,118,44,5)
            c.setFillColorRGB(.06,.18,.26); c.setFont('CN',10); c.drawString(x+8,163,label)
            if k<3: c.line(x+118,167,x+128,167)
    c.setStrokeColorRGB(.7,.8,.85); c.line(43,63,551,63)
    c.setFillColorRGB(.25,.35,.4); c.setFont('CN',9)
    c.drawString(43,43,'本材料不等于完整Demo通过；指标以唯一验收矩阵为准')
    c.drawRightString(550,43,f'{page} / {total_pages}'); c.showPage()
c.save()
(P/'docs/技术方案报告.md').write_text('\n\n'.join('# '+t+'\n\n'+'\n\n'.join(ps) for t,ps in sections),encoding='utf-8')
print(json.dumps({'pages':total_pages,'counts':{s:sum(r['status']==s for r in rows) for s in allowed_statuses}},ensure_ascii=False))
