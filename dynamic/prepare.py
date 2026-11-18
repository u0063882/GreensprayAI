"""Extract a capture-time-limited PCAP prefix using bounded HTTP ZIP ranges."""
from pathlib import Path
from collections import OrderedDict
import hashlib,io,json,struct,zipfile

class RangeReader(io.RawIOBase):
    def __init__(self,url,size,block_bytes=4*1024*1024):
        import requests
        self.url=url;self.size=size;self.position=0;self.block_bytes=block_bytes;self.cache=OrderedDict();self.session=requests.Session();self.downloaded=0
    def readable(self):return True
    def seekable(self):return True
    def tell(self):return self.position
    def seek(self,offset,whence=0):
        self.position=offset if whence==0 else self.position+offset if whence==1 else self.size+offset
        if self.position<0:raise ValueError('Negative seek')
        return self.position
    def read(self,n=-1):
        if n<0:n=self.size-self.position
        n=min(n,self.size-self.position);parts=[]
        while n>0:
            index=self.position//self.block_bytes;start=index*self.block_bytes;end=min(self.size-1,start+self.block_bytes-1)
            if index not in self.cache:
                with self.session.get(self.url,headers={'Range':f'bytes={start}-{end}'},stream=True,timeout=45) as r:
                    if r.status_code!=206 or r.headers.get('Content-Range')!=f'bytes {start}-{end}/{self.size}':raise RuntimeError('Exact HTTP range not honoured')
                    data=r.content
                if len(data)!=end-start+1:raise RuntimeError('Truncated HTTP range')
                self.cache[index]=data;self.downloaded+=len(data)
                while len(self.cache)>8:self.cache.popitem(last=False)
            data=self.cache[index];offset=self.position-start;take=min(n,len(data)-offset);parts.append(data[offset:offset+take]);self.position+=take;n-=take
        return b''.join(parts)


def trim_classic_pcap(source,target,seconds):
    import math
    if seconds is not None and (not math.isfinite(seconds) or seconds<=0):
        raise ValueError('seconds must be positive and finite, or None for the complete recording')
    header=source.read(24)
    formats={b'\xd4\xc3\xb2\xa1':('<',1e-6),b'\xa1\xb2\xc3\xd4':('>',1e-6),b'\x4d\x3c\xb2\xa1':('<',1e-9),b'\xa1\xb2\x3c\x4d':('>',1e-9)}
    if len(header)!=24 or header[:4] not in formats:raise ValueError('Expected classic PCAP; do not guess a PCAP-NG layout')
    endian,unit=formats[header[:4]];first=None;last=None;count=0;reordered=0
    with Path(target).open('wb') as f:
        f.write(header)
        while True:
            packet=source.read(16)
            if not packet:break
            if len(packet)!=16:raise ValueError('Truncated PCAP packet header')
            sec,fraction,captured,original=struct.unpack(endian+'IIII',packet);stamp=sec+fraction*unit
            if captured>16*1024*1024 or captured>original:raise ValueError('Implausible packet byte count')
            if first is None:first=stamp
            if seconds is not None and stamp-first>=seconds:break
            payload=source.read(captured)
            if len(payload)!=captured:raise ValueError('Truncated packet payload')
            if last is not None and stamp<last:reordered+=1
            f.write(packet);f.write(payload);last=stamp;count+=1
    if not count:raise ValueError('No captured packets')
    with Path(target).open('rb') as f:sha=hashlib.file_digest(f,'sha256').hexdigest()
    return dict(capture_first_unix=first,capture_last_unix=last,capture_duration_s=last-first,requested_seconds=seconds,packets=count,backward_packet_timestamps=reordered,pilot_pcap_bytes=Path(target).stat().st_size,pilot_pcap_sha256=sha)


def prepare(url,archive_bytes,output,seconds=15.):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);reader=RangeReader(url,archive_bytes)
    with zipfile.ZipFile(reader) as archive:
        captures=[x for x in archive.infolist() if x.filename.lower().endswith('.pcap')]
        if len(captures)!=1:raise ValueError('One PCAP expected')
        pc=captures[0];stem=Path(pc.filename).stem;pcap=output/(stem+'.pcap')
        for suffix in ['.json','.h5']:
            matches=[x for x in archive.infolist() if Path(x.filename).name==stem+suffix]
            if suffix=='.json' and len(matches)!=1:raise ValueError('Matching Ouster metadata required')
            if matches:
                if matches[0].file_size>2*1024*1024:raise ValueError('Unexpectedly large metadata/navigation file')
                (output/(stem+suffix)).write_bytes(archive.read(matches[0]))
        with archive.open(pc) as source:report=trim_classic_pcap(source,pcap,seconds)
    report.update(source_url=url,source_zip_bytes=archive_bytes,source_pcap_bytes=pc.file_size,source_member=pc.filename,http_bytes_fetched=reader.downloaded,full_archive_checksum_verified=False,policy='First capture-time interval only, all packet types retained. Original full input remains unchanged on GitHub. ZIP-member CRC cannot be verified for a partial PCAP prefix.')
    (output/'preparation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True);return pcap,report

def prepare_local(archive_path,output,seconds=15.,expected_sha256=None):
    archive_path=Path(archive_path);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    with archive_path.open('rb') as f:archive_sha=hashlib.file_digest(f,'sha256').hexdigest()
    if expected_sha256 and archive_sha!=expected_sha256:raise RuntimeError('Input archive SHA-256 differs from release metadata')
    with zipfile.ZipFile(archive_path) as archive:
        captures=[x for x in archive.infolist() if x.filename.lower().endswith('.pcap')]
        if len(captures)!=1:raise ValueError('One PCAP expected')
        pc=captures[0];stem=Path(pc.filename).stem;pcap=output/(stem+'.pcap')
        for suffix in ['.json','.h5']:
            matches=[x for x in archive.infolist() if Path(x.filename).name==stem+suffix]
            if suffix=='.json' and len(matches)!=1:raise ValueError('Matching metadata required')
            if matches:
                if matches[0].file_size>2*1024*1024:raise ValueError('Unexpectedly large metadata')
                (output/(stem+suffix)).write_bytes(archive.read(matches[0]))
        with archive.open(pc) as source:report=trim_classic_pcap(source,pcap,seconds)
    checksum_policy='Full ZIP SHA-256 computed'+(' and verified against expected digest' if expected_sha256 else '; no external expected digest supplied')
    report.update(source_zip_sha256=archive_sha,source_zip_bytes=archive_path.stat().st_size,source_pcap_bytes=pc.file_size,source_member=pc.filename,full_archive_checksum_verified=bool(expected_sha256),policy=checksum_policy+'. All packet types within the selected capture-time interval only. No stationary results read or changed.')
    (output/'preparation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True);return pcap,report


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--url',required=True);p.add_argument('--archive-bytes',type=int,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=15.);a=p.parse_args();prepare(a.url,a.archive_bytes,a.output,a.seconds)
