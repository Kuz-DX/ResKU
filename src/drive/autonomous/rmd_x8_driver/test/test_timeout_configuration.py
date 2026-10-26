"""Verify disabling sends B3=0, and stale acknowledgements cannot confirm it."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rmd_x8_driver'))
import rmd_x8_driver_node as module


@pytest.mark.parametrize('reply', [True, False])
def test_explicit_disable_requires_fresh_zero_reply(monkeypatch, reply):
    messages, logs = [], []
    channels=[]
    for mid,name in [(1,'left'),(2,'right')]:
        ch=NS(can_id=mid,name=name,ack=500)
        ch.apply_comm_timeout_ack=lambda value,ch=ch:setattr(ch,'ack',value)
        ch.snapshot=lambda ch=ch:dict(comm_timeout_confirmed_ms=ch.ack)
        channels.append(ch)
    def send(message):
        messages.append(message)
        if reply:
            channels[message.arbitration_id-0x141].apply_comm_timeout_ack(0)
    logger=NS(info=lambda s:logs.append(('info',s)),error=lambda s:logs.append(('error',s)))
    node=NS(comm_timeout_protection_ms=0,channels=channels,bus=NS(send=send),get_logger=lambda:logger)
    clock=iter([0.,.1,.7])
    monkeypatch.setattr(module.time,'monotonic',lambda:next(clock))
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    module.RmdX8DriverNode._configure_comm_timeout_protection(node)
    assert [m.arbitration_id for m in messages]==[0x141,0x142]
    assert all(bytes(m.data)==bytes([0xB3,0,0,0,0,0,0,0]) for m in messages)
    assert all(ch.ack==(0 if reply else None) for ch in channels)
    assert sum(level=='info' for level,_ in logs)==(2 if reply else 0)
    assert sum(level=='error' for level,_ in logs)==(0 if reply else 2)
