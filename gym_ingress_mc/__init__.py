from gym.envs.registration import register

register(
    id='Ingress-v0',
    entry_point='gym_ingress_mc.envs:IngressEnv',
)