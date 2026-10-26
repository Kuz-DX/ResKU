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
    def send(message, timeout):
        assert timeout == .05
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


@pytest.mark.parametrize('fail_zero_left', [False, True])
def test_running_watchdog_then_orderly_clear_and_restart(monkeypatch, fail_zero_left):
    channels=[];frames=[];logs=[];cancelled=[]
    for mid in (1,2):
        ch=NS(can_id=mid,name=str(mid),ack=None)
        ch.apply_comm_timeout_ack=lambda value,ch=ch:setattr(ch,'ack',value)
        ch.snapshot=lambda ch=ch:dict(comm_timeout_confirmed_ms=ch.ack)
        channels.append(ch)
    def send(msg,timeout):
        frames.append((msg.arbitration_id,bytes(msg.data)))
        if fail_zero_left and msg.arbitration_id==0x141 and msg.data[0]==0xA2:
            raise RuntimeError('one stop frame failed')
        if msg.data[0]==0xB3:
            channels[msg.arbitration_id-0x141].apply_comm_timeout_ack(int.from_bytes(msg.data[4:8],'little'))
    node=NS(comm_timeout_protection_ms=500,channels=channels,bus=NS(send=send),
            control_timer=NS(cancel=lambda:cancelled.append(True)),
            get_logger=lambda:NS(info=lambda s:logs.append(s),error=lambda s:logs.append(s)))
    node._configure_comm_timeout_protection=lambda **kw:module.RmdX8DriverNode._configure_comm_timeout_protection(node,**kw)
    monkeypatch.setattr(module.time,'monotonic',lambda:0.)
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    node._configure_comm_timeout_protection()
    assert [ch.ack for ch in channels]==[500,500]
    module.RmdX8DriverNode._prepare_motor_shutdown(node)
    assert [ch.ack for ch in channels]==[0,0]
    assert node.comm_timeout_protection_ms==500  # teardown does not change running configuration
    assert cancelled==[True]
    for mid in (0x141,0x142):
        selected=[data for arb,data in frames if arb==mid]
        assert [d[0] for d in selected]==[0xB3,0xA2,0x81,0xB3,0xA2,0x81]
        assert int.from_bytes(selected[0][4:8],'little')==500
        assert selected[3]==bytes([0xB3])+bytes(7)
    count=len(frames)
    module.RmdX8DriverNode._prepare_motor_shutdown(node)
    module.RmdX8DriverNode._control_loop(node)
    assert len(frames)==count  # duplicate cleanup and late timer cannot restart motion
    node._configure_comm_timeout_protection()
    assert [ch.ack for ch in channels]==[500,500]


def test_missing_can_is_bounded_and_idempotent():
    node=NS(bus=None,control_timer=NS(cancel=lambda:None))
    module.RmdX8DriverNode._prepare_motor_shutdown(node)
    module.RmdX8DriverNode._prepare_motor_shutdown(node)
    assert node._closing
