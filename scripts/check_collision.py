"""Native regression check; browser tests exercise the shipped WASM engine too."""
from pathlib import Path
import json
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'Panthera-HT_SDK/panthera_python/Panthera-HT_description_V1.1.0/mujoco'


def load():
    assets = {f'meshes/obj/{p.name}': p.read_bytes() for p in (SOURCE / 'meshes/obj').glob('*.obj')}
    assets.update({f'collision/{p.name}': p.read_bytes() for p in (ROOT / 'models/collision').glob('*.obj')})
    assets.update({f'contact/{p.name}': p.read_bytes() for p in (ROOT / 'models/contact').glob('*.obj')})
    model = mujoco.MjModel.from_xml_string((ROOT / 'models/panthera.xml').read_text(), assets)
    return model, mujoco.MjData(model)


def check():
    model, data = load()
    kp = np.array([110, 160, 130, 55, 35, 25])
    kd = np.array([12, 16, 13, 6, 4, 3])

    def contacts():
        return [(i, c) for i, c in enumerate(data.contact)
                if model.geom_bodyid[c.geom1] > 0 and model.geom_bodyid[c.geom2] > 0
                and c.efc_address >= 0]

    def run(target, seconds):
        max_penetration = 0
        max_force = 0
        for _ in range(round(seconds / model.opt.timestep)):
            data.ctrl[:6] = np.clip(kp * (target - data.qpos[:6]) - kd * data.qvel[:6] + data.qfrc_bias[:6],
                                   model.actuator_ctrlrange[:6, 0], model.actuator_ctrlrange[:6, 1])
            data.ctrl[6:] = np.clip(-800 * data.qpos[6:] - 8 * data.qvel[6:] + data.qfrc_bias[6:], -5, 5)
            mujoco.mj_step(model, data)
            for index, contact in contacts():
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                max_force = max(max_force, force[0])
                max_penetration = max(max_penetration, -contact.dist)
        mujoco.mj_forward(model, data)
        assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
        return dict(max_penetration=max_penetration, max_force=max_force,
                    actual=data.qpos.tolist(), contacts=len(contacts()))

    mujoco.mj_forward(model, data)
    assert not contacts(), 'Zero pose has unintended self contacts'
    home = run(np.zeros(6), 2)
    assert np.max(np.abs(data.qpos)) < 0.003, home
    blocked = run(np.array([0, 0, 0, -1.57, 0, 0]), 5)
    assert data.qpos[3] > -1.25 and blocked['max_force'] > 1, blocked
    assert blocked['max_penetration'] < 0.002, blocked
    released = run(np.zeros(6), 5)
    assert np.max(np.abs(data.qpos)) < 0.02, released
    assert not contacts(), released
    assert not np.any(data.warning.number), data.warning.number
    print(json.dumps(dict(engine=mujoco.__version__, home=home, blocked=blocked, released=released), indent=2))


if __name__ == '__main__':
    check()
