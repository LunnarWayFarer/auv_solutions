"""Воспроизводимые нарушения доставки и наблюдение доступности узлов."""

import copy
import heapq
import random


class Channel:
    def __init__(self, seed=1, delay_s=0.1, loss=0.0, jitter_s=0.0):
        if not 0 <= loss <= 1 or delay_s < 0 or jitter_s < 0:
            raise ValueError('Invalid channel parameters')
        self.random = random.Random(seed)
        self.delay_s, self.loss, self.jitter_s = delay_s, loss, jitter_s
        self.queue, self.serial = [], 0
        self.nodes, self.links = {}, {}
        self.truth = []

    def set_node(self, node, available, stamp):
        self.nodes[node] = bool(available)
        self.truth.append({'stamp_s': stamp, 'node': node,
                           'event': 'node_started' if available else 'node_stopped'})

    def set_link(self, sender, receiver, available, stamp):
        self.links[(sender, receiver)] = bool(available)
        self.truth.append({'stamp_s': stamp, 'sender': sender, 'receiver': receiver,
                           'event': 'link_connected' if available else 'link_disconnected'})

    def available(self, sender, receiver):
        return (self.nodes.get(sender, True) and self.nodes.get(receiver, True)
                and self.links.get((sender, receiver), True))

    def send(self, sender, receiver, packet, now):
        self.serial += 1
        dropped = not self.available(sender, receiver) or self.random.random() < self.loss
        self.truth.append({'stamp_s': now, 'sequence': packet['sequence'],
                           'event': 'dropped' if dropped else 'sent'})
        if not dropped:
            delay = max(0, self.delay_s + self.random.uniform(-self.jitter_s, self.jitter_s))
            heapq.heappush(self.queue, (now+delay, self.serial, sender, receiver,
                                       copy.deepcopy(packet)))

    def receive(self, now):
        output = []
        while self.queue and self.queue[0][0] <= now:
            due, _, sender, receiver, packet = heapq.heappop(self.queue)
            if self.available(sender, receiver):
                output.append({'received_s': due, 'sender': sender, 'receiver': receiver,
                               'packet': packet})
            else:
                self.truth.append({'stamp_s': due, 'sequence': packet['sequence'],
                                   'event': 'dropped_in_flight'})
        return output


class Observer:
    def __init__(self, timeout_s=2.0):
        if timeout_s <= 0:
            raise ValueError('timeout_s must be positive')
        self.timeout_s = timeout_s
        self.started, self.last, self.seen, self.unreachable = {}, {}, set(), set()
        self.events, self.latencies, self.duplicates = [], [], 0

    def expect(self, node, now):
        self.started.setdefault(node, now)

    def accept(self, delivery):
        node = delivery['sender']
        packet, now = delivery['packet'], delivery['received_s']
        key = (node, packet['sequence'])
        if key in self.seen:
            self.duplicates += 1
            return False
        self.seen.add(key)
        self.last[node] = now
        if node in self.unreachable:
            self.events.append({'stamp_s': now, 'node': node, 'event': 'node_reachable'})
            self.unreachable.remove(node)
        self.latencies.append(now-packet['sent_s'])
        return True

    def tick(self, now):
        for node, start in self.started.items():
            if now-self.last.get(node, start) > self.timeout_s and node not in self.unreachable:
                self.unreachable.add(node)
                self.events.append({'stamp_s': now, 'node': node, 'event': 'node_unreachable',
                                    'reason': 'message_timeout'})


def replay(packets, *, seed=1, delay_s=0.1, loss=0.0, jitter_s=0.0,
           node_outage=None, link_outage=None):
    """Отдельно возвращает наблюдения приёмника и разметку введённых событий."""
    if not packets:
        raise ValueError('No packets')
    channel = Channel(seed, delay_s, loss, jitter_s)
    observer = Observer()
    observer.expect('auv_01', 0.0)
    changes = []
    for outage, kind in ((node_outage, 'node'), (link_outage, 'link')):
        if outage is not None:
            a, b = outage
            if not 0 <= a < b:
                raise ValueError('Invalid outage interval')
            changes.extend([(a, kind, False), (b, kind, True)])
    changes.sort()
    pending = sorted(packets, key=lambda p: p['sent_s'])
    deliveries = []
    duration = max(p['sent_s'] for p in pending)+delay_s+jitter_s+3.0
    index, change_index = 0, 0
    for step in range(int(duration*100)+2):
        now = step/100
        while change_index < len(changes) and changes[change_index][0] <= now:
            stamp, kind, available = changes[change_index]
            if kind == 'node':
                channel.set_node('auv_01', available, stamp)
            else:
                channel.set_link('auv_01', 'monitor', available, stamp)
            change_index += 1
        while index < len(pending) and pending[index]['sent_s'] <= now:
            channel.send('auv_01', 'monitor', pending[index], pending[index]['sent_s'])
            index += 1
        for delivery in channel.receive(now):
            if observer.accept(delivery):
                deliveries.append(delivery)
        # После окончания источника канал не оценивается как отказ.
        if now <= pending[-1]['sent_s']:
            observer.tick(now)
    received = len(deliveries)
    latencies = observer.latencies
    return {'sent_count': len(pending), 'received_unique_count': received,
            'loss_pct': 100*(len(pending)-received)/len(pending),
            'latency_mean_ms': 1000*sum(latencies)/len(latencies) if latencies else None,
            'latency_max_ms': 1000*max(latencies) if latencies else None,
            'observed_events': observer.events, 'deliveries': deliveries,
            'evaluation_events': channel.truth}
